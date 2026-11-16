using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Flood fills a grid's free cells into connected regions (islands), compiled with Burst and run on a worker thread:
    /// each free cell gets the id of its region, a blocked cell 0. Flying cells connect through all 26 neighbours, as
    /// searches move. Walking fills only the cells a walker stands in, connected by the <see cref="WalkMoves"/> rules taken
    /// both ways (a drop counts as a way back up), so an island never splits two cells a walk joins: a reachable goal is
    /// never rejected. Reads its own copy of the blocking and writes into arrays it is given, so a fill allocates nothing.
    /// </summary>
    [BurstCompile]
    internal struct IslandFillJob : IJob {
        /// <summary>Per cell (flattened like <see cref="GridCollection.FlattenIndex"/>), 1 when blocked.</summary>
        [ReadOnly] public NativeArray<byte> blocked;
        /// <summary>Out: per cell, its island id (1 and up), or 0 when blocked (walking: when nobody stands there).</summary>
        public NativeArray<int> islands;
        /// <summary>Scratch, one entry per cell: the fill's frontier.</summary>
        public NativeArray<int> queue;
        public int width;
        public int height;
        public int length;

        /// <summary>Fill walking islands instead of flying ones (the walking fields below apply).</summary>
        public bool walking;
        public int clearance;
        /// <summary>Walking: the most cells one step moves up or down, either way.</summary>
        public int reachCells;

        public void Execute() {
            int cells = islands.Length;
            for(int i = 0; i < cells; i++) {
                islands[i] = 0;
            }

            if(walking) {
                FillWalking();
                return;
            }

            int plane = width * height;
            int island = 0;
            for(int seed = 0; seed < cells; seed++) {
                if(blocked[seed] != 0 || islands[seed] != 0) {
                    continue;
                }

                island++;
                islands[seed] = island;
                int head = 0;
                int tail = 0;
                queue[tail++] = seed;

                while(head < tail) {
                    int current = queue[head++];
                    int z = current / plane;
                    int rest = current - z * plane;
                    int y = rest / width;
                    int x = rest - y * width;

                    for(int dz = -1; dz <= 1; dz++) {
                        int nz = z + dz;
                        if(nz < 0 || nz >= length) {
                            continue;
                        }

                        for(int dy = -1; dy <= 1; dy++) {
                            int ny = y + dy;
                            if(ny < 0 || ny >= height) {
                                continue;
                            }

                            for(int dx = -1; dx <= 1; dx++) {
                                int nx = x + dx;
                                if(nx < 0 || nx >= width) {
                                    continue;
                                }

                                // The cell itself is already marked, so it is skipped like any visited one.
                                int next = nx + ny * width + nz * plane;
                                if(islands[next] != 0 || blocked[next] != 0) {
                                    continue;
                                }

                                islands[next] = island;
                                queue[tail++] = next;
                            }
                        }
                    }
                }
            }
        }

        private void FillWalking() {
            int3 size = new int3(width, height, length);
            int cells = islands.Length;
            int plane = width * height;
            int island = 0;
            for(int seed = 0; seed < cells; seed++) {
                if(islands[seed] != 0 || blocked[seed] != 0) {
                    continue;
                }

                int3 seedCell = Cell(seed, plane);
                if(!WalkMoves.Standable(blocked, size, seedCell, clearance)) {
                    continue;
                }

                island++;
                islands[seed] = island;
                int head = 0;
                int tail = 0;
                queue[tail++] = seed;

                while(head < tail) {
                    int3 cell = Cell(queue[head++], plane);
                    for(int dx = -1; dx <= 1; dx++) {
                        for(int dz = -1; dz <= 1; dz++) {
                            if(dx == 0 && dz == 0) {
                                continue;
                            }

                            for(int dy = -reachCells; dy <= reachCells; dy++) {
                                int3 next = cell + new int3(dx, dy, dz);
                                if(!WalkMoves.Standable(blocked, size, next, clearance)) {
                                    continue;
                                }

                                int index = WalkMoves.Flatten(size, next);
                                if(islands[index] != 0 || !WalkMoves.BodyFits(blocked, size, cell, next, clearance)) {
                                    continue;
                                }

                                islands[index] = island;
                                queue[tail++] = index;
                            }
                        }
                    }
                }
            }
        }

        private int3 Cell(int index, int plane) {
            int z = index / plane;
            int rest = index - z * plane;
            int y = rest / width;
            return new int3(rest - y * width, y, z);
        }
    }
}
