using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// The blocked cells for a body bigger than one cell (<see cref="PathBody"/>): a cell is blocked for it when any cell
    /// its body would fill is (the <see cref="up"/> cells above it, and <see cref="ring"/> cells round them each way), or
    /// when that body would stick out of the grid's sides. Above the grid's top is open: a grid ends where the level's
    /// flying space does, and the sky goes on over it (a body on a roof near the top stands with its head above it).
    /// Burst, one cell per index, on worker threads.
    /// </summary>
    [BurstCompile]
    internal struct BodyMaskJob : IJobParallelFor {
        [ReadOnly] public NativeArray<byte> blocked;
        [WriteOnly] public NativeArray<byte> body;
        public int width;
        public int height;
        public int length;
        public int up;
        public int ring;

        public void Execute(int index) {
            int x = index % width;
            int y = (index / width) % height;
            int z = index / (width * height);
            if (x - ring < 0 || x + ring >= width || z - ring < 0 || z + ring >= length) {
                body[index] = 1;
                return;
            }

            int layer = width * height;
            int top = up < height - 1 - y ? up : height - 1 - y;
            for (int dy = 0; dy <= top; dy++) {
                for (int dz = -ring; dz <= ring; dz++) {
                    int row = (y + dy) * width + (z + dz) * layer;
                    for (int dx = -ring; dx <= ring; dx++) {
                        if (blocked[row + x + dx] != 0) {
                            body[index] = 1;
                            return;
                        }
                    }
                }
            }

            body[index] = 0;
        }
    }
}
