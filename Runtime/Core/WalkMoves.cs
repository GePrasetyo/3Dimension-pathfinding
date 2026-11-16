using Unity.Collections;
using Unity.Mathematics;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// The walking rules over a grid's blocked cells, shared by the searches and the island fills (Burst compiles them
    /// into each job). Above the grid's top counts as open sky; below its bottom there is no floor.
    /// </summary>
    internal static class WalkMoves {
        /// <summary>Free to be in: inside the grid and not blocked, or above its top.</summary>
        public static bool Free(in NativeArray<byte> blocked, int3 size, int3 cell) {
            if (cell.y >= size.y) {
                return cell.x >= 0 && cell.x < size.x && cell.z >= 0 && cell.z < size.z;
            }

            if (math.any(cell < 0) || cell.x >= size.x || cell.z >= size.z) {
                return false;
            }

            return blocked[Flatten(size, cell)] == 0;
        }

        /// <summary>The cells of a column from <paramref name="fromY"/> up to <paramref name="toY"/> are all free.</summary>
        public static bool ColumnFree(in NativeArray<byte> blocked, int3 size, int x, int z, int fromY, int toY) {
            for (int y = fromY; y <= toY; y++) {
                if (!Free(blocked, size, new int3(x, y, z))) {
                    return false;
                }
            }

            return true;
        }

        /// <summary>A walker can stand here: free, a blocked cell under it, and <paramref name="clearance"/> free cells from it up.</summary>
        public static bool Standable(in NativeArray<byte> blocked, int3 size, int3 cell, int clearance) {
            if (cell.y < 1 || math.any(cell < 0) || cell.x >= size.x || cell.y >= size.y || cell.z >= size.z) {
                return false;
            }

            if (blocked[Flatten(size, cell - new int3(0, 1, 0))] == 0) {
                return false;
            }

            return ColumnFree(blocked, size, cell.x, cell.z, cell.y, cell.y + clearance - 1);
        }

        /// <summary>
        /// A walker standing at <paramref name="from"/> fits through the move to the standable <paramref name="to"/> in the
        /// next column: both columns are free from their floor up to the higher head, and a diagonal move doesn't squeeze
        /// past a corner (both columns beside it are free over the height it sweeps). The same both ways.
        /// </summary>
        public static bool BodyFits(in NativeArray<byte> blocked, int3 size, int3 from, int3 to, int clearance) {
            int top = math.max(from.y, to.y) + clearance - 1;
            if (!ColumnFree(blocked, size, from.x, from.z, from.y, top) || !ColumnFree(blocked, size, to.x, to.z, to.y, top)) {
                return false;
            }

            int dx = to.x - from.x;
            int dz = to.z - from.z;
            if (dx != 0 && dz != 0) {
                int low = math.min(from.y, to.y);
                if (!ColumnFree(blocked, size, from.x + dx, from.z, low, top) || !ColumnFree(blocked, size, from.x, from.z + dz, low, top)) {
                    return false;
                }
            }

            return true;
        }

        public static int Flatten(int3 size, int3 cell) => cell.x + cell.y * size.x + cell.z * size.x * size.y;
    }
}
