using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// One pass of the blocked cells for a body bigger than one cell (<see cref="PathBody"/>): along one axis, a cell is
    /// blocked when any cell from <see cref="before"/> behind it to <see cref="after"/> ahead of it is. A body's cells are a
    /// box, so three passes make them, one per axis (<see cref="GridSearchData.BodyBlocked"/>): across x and z for its
    /// ring, both ways and out of the grid's sides counting as blocked (<see cref="edgeBlocked"/>); then up y for its
    /// height, ahead only and the open sky above the grid's top counting as free (a grid ends where the level's flying
    /// space does, and the sky goes on over it: a body on a roof near the top stands with its head above it).
    /// Each pass reads every cell once or twice whatever the body's size: one line of cells per index, a running count
    /// of the blocked ones in the window as it slides. Burst, on worker threads.
    /// </summary>
    [BurstCompile]
    internal struct BodyMaskJob : IJobParallelFor {
        [ReadOnly] public NativeArray<byte> source;
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<byte> target;
        /// <summary>Cells along one line.</summary>
        public int count;
        /// <summary>Index step from a cell to the next along the line.</summary>
        public int step;
        /// <summary>Line starts: line <c>i</c> starts at <c>(i % inner) * innerStep + (i / inner) * outerStep</c>.</summary>
        public int inner;
        public int innerStep;
        public int outerStep;
        /// <summary>The window: cells behind and ahead of the cell, along the line.</summary>
        public int before;
        public int after;
        /// <summary>A window reaching out of the grid blocks the cell (the sides), or the cells out there count as free (the sky).</summary>
        public bool edgeBlocked;

        public void Execute(int line) {
            int start = line % inner * innerStep + line / inner * outerStep;
            // Blocked cells in the first cell's window.
            int blocked = 0;
            int first = after < count - 1 ? after : count - 1;
            for (int i = 0; i <= first; i++) {
                blocked += source[start + i * step];
            }

            for (int i = 0; i < count; i++) {
                bool outside = i - before < 0 || i + after >= count;
                target[start + i * step] = blocked > 0 || (edgeBlocked && outside) ? (byte)1 : (byte)0;
                int enters = i + after + 1;
                if (enters < count) {
                    blocked += source[start + enters * step];
                }

                int leaves = i - before;
                if (leaves >= 0) {
                    blocked -= source[start + leaves * step];
                }
            }
        }
    }
}
