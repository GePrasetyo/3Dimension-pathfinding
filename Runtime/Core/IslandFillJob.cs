using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Flood fills a grid's free cells into connected regions (islands), compiled with Burst and run on a worker thread:
    /// each free cell gets the id of its region, a blocked cell 0. Cells connect through all 26 neighbours, as searches
    /// move. Reads its own copy of the blocking and writes into arrays it is given, so a fill allocates nothing.
    /// </summary>
    [BurstCompile]
    internal struct IslandFillJob : IJob {
        /// <summary>Per cell (flattened like <see cref="GridCollection.FlattenIndex"/>), 1 when blocked.</summary>
        [ReadOnly] public NativeArray<byte> blocked;
        /// <summary>Out: per cell, its island id (1 and up), or 0 when blocked.</summary>
        public NativeArray<int> islands;
        /// <summary>Scratch, one entry per cell: the fill's frontier.</summary>
        public NativeArray<int> queue;
        public int width;
        public int height;
        public int length;

        public void Execute() {
            int cells = islands.Length;
            for(int i = 0; i < cells; i++) {
                islands[i] = 0;
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
    }
}
