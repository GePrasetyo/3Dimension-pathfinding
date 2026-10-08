using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// What searches over one grid run on: a native mirror of the grid's blocked cells (refreshed when obstacles
    /// change, never while a search is reading it) and reusable scratch arrays, one set per search in flight. Native
    /// memory, so the grid disposes it (<see cref="GridCollection.Dispose"/>).
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

        private readonly GridCollection collection;
        private readonly Stack<Scratch> free = new Stack<Scratch>();
        private readonly List<Scratch> all = new List<Scratch>();
        private NativeArray<byte> blocked;
        private int version;
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
            stale = false;
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
            if (blocked.IsCreated) {
                blocked.Dispose();
            }
        }
    }
}
