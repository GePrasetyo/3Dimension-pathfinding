using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Weighted A* over a grid's blocked-cell mirror, compiled with Burst and run on a worker thread (or inline for a
    /// synchronous query): f = g + weight * euclidean, no reopening. Flying moves through all 26 neighbours without
    /// cutting corners; walking moves between cells a walker stands in, to the 8 columns around, by the
    /// <see cref="WalkMoves"/> rules. Scratch arrays are reused across searches through stamping, so a search never clears them.
    /// </summary>
    [BurstCompile]
    internal struct AStarJob : IJob {
        public const int Finished = 1;
        public const int NotFound = 0;

        private const int NotInOpenSet = -1;
        private const int Closed = -2;

        [ReadOnly] public NativeArray<byte> blocked;
        public int width;
        public int height;
        public int length;
        public float pointDistance;
        public float heuristicWeight;
        public int start;
        public int goal;
        /// <summary>Upper bound on expanded cells (keeps a worker from being held by a hopeless search).</summary>
        public int maxExpansions;

        /// <summary>Walk instead of fly (the walking fields below apply).</summary>
        public bool walking;
        /// <summary>Walking: free cells the walker needs from the floor up.</summary>
        public int clearance;
        /// <summary>Walking: cells it climbs in one step.</summary>
        public int stepCells;
        /// <summary>Walking: cells it drops in one step.</summary>
        public int dropCells;

        public NativeArray<float> gScore;
        public NativeArray<float> fScore;
        public NativeArray<int> cameFrom;
        public NativeArray<int> stamp;
        public NativeArray<int> heapIndex;
        public NativeArray<int> heap;
        public int stampId;

        /// <summary>The path from goal back to start, as flattened cell indices.</summary>
        public NativeList<int> path;
        /// <summary>[0]: <see cref="Finished"/> or <see cref="NotFound"/>.</summary>
        public NativeArray<int> result;

        private int heapCount;

        public void Execute() {
            path.Clear();
            result[0] = NotFound;
            heapCount = 0;

            int3 goalCell = Cell(goal);
            Touch(start);
            gScore[start] = 0f;
            fScore[start] = heuristicWeight * math.distance(Cell(start), goalCell) * pointDistance;
            cameFrom[start] = -1;
            Push(start);

            int expansions = 0;
            while (heapCount > 0) {
                int current = PopMin();
                if (current == goal) {
                    for (int node = current, guard = blocked.Length; node != -1 && guard > 0; node = cameFrom[node], guard--) {
                        path.Add(node);
                    }

                    result[0] = Finished;
                    return;
                }

                if (++expansions > maxExpansions) {
                    return;
                }

                int3 cell = Cell(current);
                float currentG = gScore[current];
                if (walking) {
                    ExpandWalking(current, cell, currentG, goalCell);
                    continue;
                }

                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }

                            int3 next = cell + new int3(dx, dy, dz);
                            if (math.any(next < 0) || next.x >= width || next.y >= height || next.z >= length) {
                                continue;
                            }

                            int index = Flatten(next);
                            if (blocked[index] != 0 || !CornersFree(cell, dx, dy, dz)) {
                                continue;
                            }

                            Relax(current, index, next, currentG + pointDistance * math.sqrt(dx * dx + dy * dy + dz * dz), goalCell);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Walking: from a cell the walker stands in to every standable cell in the 8 columns around it, from a step up to a
        /// drop down, where its body fits through the move.
        /// </summary>
        private void ExpandWalking(int current, int3 cell, float currentG, int3 goalCell) {
            int3 size = new int3(width, height, length);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dz == 0) {
                        continue;
                    }

                    for (int dy = stepCells; dy >= -dropCells; dy--) {
                        int3 next = cell + new int3(dx, dy, dz);
                        if (!WalkMoves.Standable(blocked, size, next, clearance) || !WalkMoves.BodyFits(blocked, size, cell, next, clearance)) {
                            continue;
                        }

                        Relax(current, Flatten(next), next, currentG + pointDistance * math.sqrt(dx * dx + dy * dy + dz * dz), goalCell);
                    }
                }
            }
        }

        /// <summary>Reaches <paramref name="index"/> from <paramref name="current"/> at cost <paramref name="tentative"/>, if that is the cheapest way yet.</summary>
        private void Relax(int current, int index, int3 next, float tentative, int3 goalCell) {
            if (stamp[index] != stampId) {
                Touch(index);
            } else if (heapIndex[index] == Closed) {
                return;
            }

            if (tentative < gScore[index]) {
                cameFrom[index] = current;
                gScore[index] = tentative;
                fScore[index] = tentative + heuristicWeight * math.distance(next, goalCell) * pointDistance;
                if (heapIndex[index] == NotInOpenSet) {
                    Push(index);
                } else {
                    SiftUp(heapIndex[index]);
                }
            }
        }

        /// <summary>A diagonal step is passable only when the cells it squeezes past are free too.</summary>
        private bool CornersFree(int3 cell, int dx, int dy, int dz) {
            int axes = math.abs(dx) + math.abs(dy) + math.abs(dz);
            if (axes < 2) {
                return true;
            }

            for (int mask = 1; mask < 7; mask++) {
                int3 sub = new int3((mask & 1) != 0 ? dx : 0, (mask & 2) != 0 ? dy : 0, (mask & 4) != 0 ? dz : 0);
                int subAxes = math.abs(sub.x) + math.abs(sub.y) + math.abs(sub.z);
                if (subAxes == 0 || subAxes == axes) {
                    continue;
                }

                if (blocked[Flatten(cell + sub)] != 0) {
                    return false;
                }
            }

            return true;
        }

        private void Touch(int index) {
            stamp[index] = stampId;
            gScore[index] = float.PositiveInfinity;
            heapIndex[index] = NotInOpenSet;
        }

        private int3 Cell(int index) => new int3(index % width, (index / width) % height, index / (width * height));

        private int Flatten(int3 cell) => cell.x + cell.y * width + cell.z * width * height;

        private void Push(int node) {
            int index = heapCount++;
            heap[index] = node;
            heapIndex[node] = index;
            SiftUp(index);
        }

        private int PopMin() {
            int root = heap[0];
            heapIndex[root] = Closed;
            heapCount--;
            if (heapCount > 0) {
                int last = heap[heapCount];
                heap[0] = last;
                heapIndex[last] = 0;
                SiftDown(0);
            }

            return root;
        }

        private void SiftUp(int index) {
            int node = heap[index];
            float score = fScore[node];
            while (index > 0) {
                int parentIndex = (index - 1) / 2;
                int parent = heap[parentIndex];
                if (score >= fScore[parent]) {
                    break;
                }

                heap[index] = parent;
                heapIndex[parent] = index;
                index = parentIndex;
            }

            heap[index] = node;
            heapIndex[node] = index;
        }

        private void SiftDown(int index) {
            int node = heap[index];
            float score = fScore[node];
            while (true) {
                int left = 2 * index + 1;
                if (left >= heapCount) {
                    break;
                }

                int smallest = left;
                int right = left + 1;
                if (right < heapCount && fScore[heap[right]] < fScore[heap[left]]) {
                    smallest = right;
                }

                if (fScore[heap[smallest]] >= score) {
                    break;
                }

                heap[index] = heap[smallest];
                heapIndex[heap[index]] = index;
                index = smallest;
            }

            heap[index] = node;
            heapIndex[node] = index;
        }
    }
}
