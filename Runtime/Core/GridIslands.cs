using System;
using Unity.Collections;
using Unity.Jobs;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// A grid's island maps: which connected region every cell belongs to, for O(1) reachability checks. The static map
    /// counts baked blocking only; the dynamic map counts obstacles too and is trusted only while its stamp matches
    /// <see cref="GridLocator.obstacleVersion"/>. Every fill is an <see cref="IslandFillJob"/>. The dynamic refill runs in
    /// the background: it fills a second map on a worker thread, from its own copy of the blocking, while queries keep
    /// reading the last complete one, and the two swap once it is done. The maps are native and reused, so a refill
    /// allocates nothing; the grid releases them (<see cref="GridCollection.Dispose"/>).
    /// </summary>
    internal sealed class GridIslands : IDisposable {
        private readonly GridCollection collection;
        private NativeArray<int> staticMap;
        private NativeArray<int> dynamicMap;
        /// <summary>What the dynamic fill in flight writes; becomes <see cref="dynamicMap"/> when it is done.</summary>
        private NativeArray<int> filling;
        /// <summary>The blocking a fill reads (its own copy: the search mirror stays free to change meanwhile).</summary>
        private NativeArray<byte> mask;
        private NativeArray<int> queue;
        private JobHandle fill;
        private bool running;
        private int fillVersion;
        private int dynamicVersion = -1;
        private float nextFill;

        public GridIslands(GridCollection collection) {
            this.collection = collection;
        }

        private int Cells => collection.gridWidth * collection.gridHeight * collection.gridLength;

        public bool HasStatic => staticMap.IsCreated && staticMap.Length == Cells;

        public bool HasFreshDynamic => dynamicMap.IsCreated && dynamicMap.Length == Cells && dynamicVersion == GridLocator.obstacleVersion;

        public int StaticIsland(int index) => staticMap[index];

        public int DynamicIsland(int index) => dynamicMap[index];

        /// <summary>Fills the static map now, from baked blocking only; the dynamic map is stale after it.</summary>
        public void ComputeStatic() {
            Prepare();
            GridNode[][][] grids = collection.grids;
            int width = collection.gridWidth;
            int height = collection.gridHeight;
            for(int x = 0; x < width; x++) {
                for(int y = 0; y < height; y++) {
                    GridNode[] column = grids[x][y];
                    for(int z = 0; z < column.Length; z++) {
                        mask[x + y * width + z * width * height] = column[z].staticInvalid ? (byte)1 : (byte)0;
                    }
                }
            }

            Schedule(staticMap).Complete();
            dynamicVersion = -1;
        }

        /// <summary>Fills the dynamic map now, from current blocking (static and dynamic).</summary>
        public void ComputeDynamic() {
            Prepare();
            int version = CopyBlocking();
            Schedule(dynamicMap).Complete();
            dynamicVersion = version;
        }

        /// <summary>
        /// Background upkeep, once a frame: swaps in a finished refill, and starts one when the dynamic map is stale and
        /// <paramref name="cooldown"/> seconds have passed since the last start.
        /// </summary>
        public void Tick(float time, float cooldown) {
            if(running) {
                if(!fill.IsCompleted) {
                    return;
                }

                fill.Complete();
                running = false;
                (dynamicMap, filling) = (filling, dynamicMap);
                dynamicVersion = fillVersion;
            }

            if(HasFreshDynamic || time < nextFill) {
                return;
            }

            nextFill = time + cooldown;
            Prepare();
            fillVersion = CopyBlocking();
            fill = Schedule(filling);
            JobHandle.ScheduleBatchedJobs();
            running = true;
        }

        public void Dispose() {
            fill.Complete();
            running = false;
            Release();
        }

        /// <summary>Ends any fill in flight and sizes the arrays to the grid (they keep their contents while it fits).</summary>
        private void Prepare() {
            if(running) {
                fill.Complete();
                running = false;
            }

            int cells = Cells;
            if(staticMap.IsCreated && staticMap.Length == cells) {
                return;
            }

            Release();
            staticMap = new NativeArray<int>(cells, Allocator.Persistent);
            dynamicMap = new NativeArray<int>(cells, Allocator.Persistent);
            filling = new NativeArray<int>(cells, Allocator.Persistent);
            mask = new NativeArray<byte>(cells, Allocator.Persistent);
            queue = new NativeArray<int>(cells, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            dynamicVersion = -1;
        }

        /// <summary>Copies current blocking (the search mirror, brought up to date) into the fill's mask; the obstacle version it is.</summary>
        private int CopyBlocking() {
            GridSearchData search = collection.SearchData;
            search.Refresh();
            NativeArray<byte>.Copy(search.Blocked, mask);
            return GridLocator.obstacleVersion;
        }

        private JobHandle Schedule(NativeArray<int> into) {
            return new IslandFillJob {
                blocked = mask,
                islands = into,
                queue = queue,
                width = collection.gridWidth,
                height = collection.gridHeight,
                length = collection.gridLength
            }.Schedule();
        }

        private void Release() {
            if(staticMap.IsCreated) {
                staticMap.Dispose();
                dynamicMap.Dispose();
                filling.Dispose();
                mask.Dispose();
                queue.Dispose();
            }
        }
    }
}
