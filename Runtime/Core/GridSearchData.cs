using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// What searches over one grid run on: a native mirror of the grid's blocked cells (refreshed when obstacles
    /// change, never while a search is reading it), the blocked cells for each body size searched (<see cref="PathBody"/>,
    /// made from the mirror when a search asks after it changed) and reusable scratch arrays, one set per search in
    /// flight. Native memory, so the grid disposes it (<see cref="GridCollection.Dispose"/>).
    /// </summary>
    internal sealed class GridSearchData : IDisposable {
        internal sealed class Scratch : IDisposable {
            public NativeArray<float> gScore;
            public NativeArray<float> fScore;
            public NativeArray<int> cameFrom;
            public NativeArray<int> stamp;
            public NativeArray<int> heapIndex;
            public NativeArray<int> heap;
            public NativeList<int> path;
            public NativeArray<int> result;
            public int stampId;

            public Scratch(int cells) {
                gScore = new NativeArray<float>(cells, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                fScore = new NativeArray<float>(cells, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                cameFrom = new NativeArray<int>(cells, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                stamp = new NativeArray<int>(cells, Allocator.Persistent);
                heapIndex = new NativeArray<int>(cells, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                heap = new NativeArray<int>(cells, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                path = new NativeList<int>(64, Allocator.Persistent);
                result = new NativeArray<int>(1, Allocator.Persistent);
            }

            /// <summary>A fresh stamp for the next search (stamps from earlier searches then read as untouched).</summary>
            public int NextStamp() {
                if (stampId == int.MaxValue) {
                    for (int i = 0; i < stamp.Length; i++) {
                        stamp[i] = 0;
                    }

                    stampId = 0;
                }

                return ++stampId;
            }

            public void Dispose() {
                gScore.Dispose();
                fScore.Dispose();
                cameFrom.Dispose();
                stamp.Dispose();
                heapIndex.Dispose();
                heap.Dispose();
                path.Dispose();
                result.Dispose();
            }
        }

        /// <summary>The blocked cells for one body size, as of mirror <see cref="version"/>.</summary>
        private sealed class BodyMask {
            public readonly int up;
            public readonly int ring;
            public NativeArray<byte> blocked;
            public int version = -1;

            public BodyMask(int up, int ring) {
                this.up = up;
                this.ring = ring;
            }
        }

        private readonly GridCollection collection;
        private readonly List<BodyMask> bodies = new List<BodyMask>();
        /// <summary>Between the passes that make a body's cells (<see cref="BodyBlocked"/>), one for all bodies.</summary>
        private NativeArray<byte> bodyScratch;
        private readonly Stack<Scratch> free = new Stack<Scratch>();
        private readonly List<Scratch> all = new List<Scratch>();
        private NativeArray<byte> blocked;
        private int version;
        /// <summary>Counts the mirror's rewrites: a body's cells made before the latest are out of date.</summary>
        private int mirrorVersion;
        private bool stale = true;
        private JobHandle readers;
        private bool disposed;

        public GridSearchData(GridCollection collection) {
            this.collection = collection;
        }

        public NativeArray<byte> Blocked => blocked;
        public bool IsDisposed => disposed;

        /// <summary>The grid's cells were replaced or rebaked: the mirror is rebuilt before the next search.</summary>
        public void Invalidate() {
            stale = true;
        }

        /// <summary>Brings the mirror up to date with the grid (waits for searches still reading the old one).</summary>
        public void Refresh() {
            if (disposed) {
                throw new ObjectDisposedException(nameof(GridSearchData));
            }

            int cells = collection.gridWidth * collection.gridHeight * collection.gridLength;
            if (!stale && blocked.IsCreated && blocked.Length == cells && version == GridLocator.obstacleVersion) {
                return;
            }

            readers.Complete();
            readers = default;
            if (!blocked.IsCreated || blocked.Length != cells) {
                if (blocked.IsCreated) {
                    blocked.Dispose();
                }

                blocked = new NativeArray<byte>(cells, Allocator.Persistent);
                foreach (Scratch scratch in all) {
                    scratch.Dispose();
                }

                all.Clear();
                free.Clear();
                DisposeBodies();
            }

            GridNode[][][] grids = collection.grids;
            int width = collection.gridWidth;
            int height = collection.gridHeight;
            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    GridNode[] column = grids[x][y];
                    for (int z = 0; z < column.Length; z++) {
                        blocked[x + y * width + z * width * height] = column[z].invalid ? (byte)1 : (byte)0;
                    }
                }
            }

            version = GridLocator.obstacleVersion;
            mirrorVersion++;
            stale = false;
        }

        /// <summary>
        /// The blocked cells for a body <paramref name="up"/> cells tall above its lowest and <paramref name="ring"/> cells
        /// round (<see cref="PathBody.Cells"/>), as of the mirror's last <see cref="Refresh"/>: the mirror itself for a body
        /// of one cell; otherwise made again (Burst jobs, a pass per axis, waiting for searches still reading the old) when the mirror changed.
        /// </summary>
        public NativeArray<byte> BodyBlocked(int up, int ring) {
            if (up <= 0 && ring <= 0) {
                return blocked;
            }

            BodyMask mask = null;
            foreach (BodyMask candidate in bodies) {
                if (candidate.up == up && candidate.ring == ring) {
                    mask = candidate;
                    break;
                }
            }

            if (mask == null) {
                mask = new BodyMask(up, ring);
                bodies.Add(mask);
            }

            if (mask.version == mirrorVersion && mask.blocked.IsCreated && mask.blocked.Length == blocked.Length) {
                return mask.blocked;
            }

            readers.Complete();
            readers = default;
            if (!mask.blocked.IsCreated || mask.blocked.Length != blocked.Length) {
                if (mask.blocked.IsCreated) {
                    mask.blocked.Dispose();
                }

                mask.blocked = new NativeArray<byte>(blocked.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            // A body's cells are a box: a pass across x and one across z for its ring, then one up y for its height, each
            // into the other array, so the last lands in the mask.
            int width = collection.gridWidth;
            int height = collection.gridHeight;
            int length = collection.gridLength;
            int layer = width * height;
            int passes = (ring > 0 ? 2 : 0) + (up > 0 ? 1 : 0);
            if (passes > 1 && (!bodyScratch.IsCreated || bodyScratch.Length != blocked.Length)) {
                if (bodyScratch.IsCreated) {
                    bodyScratch.Dispose();
                }

                bodyScratch = new NativeArray<byte>(blocked.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            NativeArray<byte> source = blocked;
            JobHandle done = default;
            if (ring > 0) {
                // Across x: the rows, one per (y, z).
                done = Pass(ref source, ref passes, mask, new BodyMaskJob {
                    count = width, step = 1, inner = height * length, innerStep = width, outerStep = 0,
                    before = ring, after = ring, edgeBlocked = true
                }, height * length, done);
                // Across z: one line per (x, y).
                done = Pass(ref source, ref passes, mask, new BodyMaskJob {
                    count = length, step = layer, inner = layer, innerStep = 1, outerStep = 0,
                    before = ring, after = ring, edgeBlocked = true
                }, layer, done);
            }

            if (up > 0) {
                // Up y: one column per (x, z).
                done = Pass(ref source, ref passes, mask, new BodyMaskJob {
                    count = height, step = width, inner = width, innerStep = 1, outerStep = layer,
                    before = 0, after = up, edgeBlocked = false
                }, width * length, done);
            }

            done.Complete();
            mask.version = mirrorVersion;
            return mask.blocked;
        }

        /// <summary>
        /// Schedules one pass of a body's cells from <paramref name="source"/> into the mask when it is the last of the
        /// <paramref name="left"/> passes, into the scratch otherwise (one apart, so the last lands in the mask), and makes
        /// its target the next pass's source.
        /// </summary>
        private JobHandle Pass(ref NativeArray<byte> source, ref int left, BodyMask mask, BodyMaskJob pass, int lines, JobHandle after) {
            left--;
            NativeArray<byte> target = left % 2 == 0 ? mask.blocked : bodyScratch;
            pass.source = source;
            pass.target = target;
            source = target;
            return pass.Schedule(lines, 64, after);
        }

        private void DisposeBodies() {
            foreach (BodyMask mask in bodies) {
                if (mask.blocked.IsCreated) {
                    mask.blocked.Dispose();
                }
            }

            bodies.Clear();
            if (bodyScratch.IsCreated) {
                bodyScratch.Dispose();
            }
        }

        public Scratch Rent() {
            if (free.Count > 0) {
                return free.Pop();
            }

            var scratch = new Scratch(blocked.Length);
            all.Add(scratch);
            return scratch;
        }

        public void Return(Scratch scratch) {
            if (!disposed && all.Contains(scratch)) {
                free.Push(scratch);
            }
        }

        /// <summary>A search reads the mirror until <paramref name="handle"/> completes.</summary>
        public void AddReader(JobHandle handle) {
            readers = JobHandle.CombineDependencies(readers, handle);
        }

        public void Dispose() {
            if (disposed) {
                return;
            }

            disposed = true;
            readers.Complete();
            foreach (Scratch scratch in all) {
                scratch.Dispose();
            }

            all.Clear();
            free.Clear();
            DisposeBodies();
            if (blocked.IsCreated) {
                blocked.Dispose();
            }
        }
    }
}
