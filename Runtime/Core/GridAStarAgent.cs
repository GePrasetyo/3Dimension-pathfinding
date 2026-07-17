using Cysharp.Threading.Tasks;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    public class GridAStarAgent {
        public PathAgentType agentType { get; }

        protected Transform agentTransform;

        /// <summary>
        /// Weighted A*: f = g + weight * euclideanDistance. Weight 1 gives optimal paths,
        /// higher values expand fewer nodes with paths at most weight times longer.
        /// </summary>
        public float heuristicWeight = 2f;

        /// <summary>
        /// Line-of-sight pruning of grid waypoints so agents fly straight lines
        /// instead of tracing cell centers. Corner-adjacent waypoints are kept so
        /// spline or follower smoothing stays pinned near obstacles.
        /// </summary>
        public bool prunePathWaypoints = true;

        // Cap so follower smoothing stays bounded on long straights.
        private const float maxPrunedSegmentLength = 4f;

        private const int maxExpansionsPerFrame = 1000;

        // heapIndex states for a node stamped by the current search
        private const int notInOpenSet = -1;
        private const int closedNode = -2;

        private GridCollection gridCollection;
        private bool isOn;
        private bool debug;

        private List<GridNode> tempPath = new List<GridNode>();

        public ReentryCanceller reentryCanceller = new();

        /// <summary>
        /// Reusable per-search scratch data. Nodes are lazily reset through searchId
        /// stamping, so starting a search never has to clear or reallocate the arrays.
        /// Contexts are pooled so a synchronous search can run while a time-sliced
        /// asynchronous search is suspended without corrupting its state.
        /// </summary>
        private class SearchContext {
            public float[] gScore;
            public float[] fScore;
            public int[] cameFrom;
            public int[] searchStamp;
            public int[] heapIndex;
            public int[] heap;
            public int heapCount;
            public int searchId;
            public bool busy;

            public void EnsureCapacity(int totalSize) {
                if(gScore == null || gScore.Length < totalSize) {
                    gScore = new float[totalSize];
                    fScore = new float[totalSize];
                    cameFrom = new int[totalSize];
                    searchStamp = new int[totalSize];
                    heapIndex = new int[totalSize];
                    heap = new int[totalSize];
                    searchId = 0;
                }
            }
        }

        private static List<SearchContext> contextPool = new List<SearchContext>();

        private static SearchContext RentContext(int totalSize) {
            SearchContext context = null;

            for(int i = 0; i < contextPool.Count; i++) {
                if(!contextPool[i].busy) {
                    context = contextPool[i];
                    break;
                }
            }

            if(context == null) {
                context = new SearchContext();
                contextPool.Add(context);
            }

            context.EnsureCapacity(totalSize);
            context.busy = true;
            context.searchId++;
            context.heapCount = 0;
            return context;
        }

        public GridAStarAgent(Transform transform, PathAgentType type, bool debugFlag = false) {
            agentTransform = transform;
            isOn = true;
            agentType = type;
            debug = debugFlag;
        }

        public async void AgentState(bool on) {
            if(!on) {
                reentryCanceller.Dispose();
                isOn = on;
                gridCollection = null;
                return;
            }

            var internalToken = await reentryCanceller.Enter(default);
            gridCollection = null;

            while(gridCollection == null) {
                //Looking for an active grid of this agent type
                gridCollection = GridLocator.GetActive(agentType);
                bool isCanceled = await UniTask.Yield(cancellationToken: internalToken).SuppressCancellationThrow();

                if(isCanceled) { break; }
            }

            reentryCanceller.Exit();

            if(on) {
                isOn = on;
            }
        }

        public bool GetPath(Vector3 startLocation, Vector3 targetLocation, ref List<Vector3> pathPoints) {
            GridCollection collection = gridCollection;

            // The non time-sliced core never awaits, so this completes synchronously.
            PathfindingStatus status = PathfindingCore(startLocation, targetLocation, tempPath, timeSliced: false, CancellationToken.None).GetAwaiter().GetResult();

            if(status == PathfindingStatus.Invalid) {
                return false;
            }

            pathPoints.Clear();
            for(int i = tempPath.Count - 1; i > 0; i--) {
                pathPoints.Add(tempPath[i].worldPosition);
            }

            PrunePath(pathPoints, collection);
            SetPathColor(pathPoints);
            return true;
        }

        public bool GetClosestStandablePosition(ref Vector3 position, float radiusCheck = 2f) {
            GridNode result = null;
            gridCollection?.GetClosestPointWorldSpace(position, out result, true, Mathf.CeilToInt(radiusCheck));

            if(result?.invalid != false) {
                return false;
            }

            position = result.worldPosition;
            return true;
        }

        public async UniTask<(bool success, List<Vector3> pathPoints)> GetPathAsync(Vector3 targetLocation, CancellationToken ct = default) {
            // Captured before the awaits so pruning runs against the same collection the
            // search used, even if the agent is toggled or rebound mid-search.
            GridCollection collection = gridCollection;
            List<Vector3> pathPoints = new List<Vector3>();
            List<GridNode> gridPath = new List<GridNode>();
            PathfindingStatus status = await PathfindingCore(agentTransform.position, targetLocation, gridPath, timeSliced: true, ct);

            if(status == PathfindingStatus.Invalid) {
                return (success: false, pathPoints);
            }

            for(int i = gridPath.Count - 1; i > 0; i--) {
                pathPoints.Add(gridPath[i].worldPosition);
            }

            PrunePath(pathPoints, collection);
            SetPathColor(pathPoints);
            return (success: true, pathPoints);
        }

        public bool IsPathValid(Vector3 goal) {
            if(gridCollection == null) {
                return false;
            }

            if(!gridCollection.CheckValidityLocation(goal)) {
                return false;
            }

            return true;
        }

        private async UniTask<PathfindingStatus> PathfindingCore(Vector3 startLocation, Vector3 goal, List<GridNode> resultPath, bool timeSliced, CancellationToken ct) {
            resultPath.Clear();

            GridCollection collection = gridCollection;
            if(collection == null || collection.grids == null) {
                return PathfindingStatus.Invalid;
            }

            collection.GetClosestPointWorldSpace(startLocation, out GridNode start);

            if(!collection.CheckValidityLocation(goal)) {
                return PathfindingStatus.Invalid;
            }

            collection.GetClosestPointWorldSpace(goal, out GridNode end, getClosestValid: false);

            if(start == end || start.invalid || end.invalid) {
                return PathfindingStatus.Invalid;
            }

            if(!collection.AreConnected(start, end)) {
                return PathfindingStatus.Invalid;
            }

            // While the dynamic island map is stale (an obstacle changed within the last
            // refresh window), a cheap reverse probe fast-fails goals that are provably
            // sealed inside a small pocket instead of exhausting the whole region.
            if(!collection.HasFreshDynamicIslands() && collection.GoalPocketSealed(end, start)) {
                return PathfindingStatus.Invalid;
            }

            int width = collection.gridWidth;
            int height = collection.gridHeight;
            int length = collection.gridLength;
            GridNode[][][] grids = collection.grids;
            float pointDistance = collection.pointDistance;
            Vector3 endPosition = end.worldPosition;
            int endIndex = collection.FlattenIndex(end.coords);
            int startIndex = collection.FlattenIndex(start.coords);

            Vector3Int[] offsets = GridCollection.neighbourOffsets;
            float[] costFactors = GridCollection.neighbourCostFactors;
            Vector3Int[][] intermediates = GridCollection.neighbourIntermediates;

            SearchContext context = RentContext(width * height * length);

            try {
                context.searchStamp[startIndex] = context.searchId;
                context.gScore[startIndex] = 0;
                context.fScore[startIndex] = heuristicWeight * Vector3.Distance(start.worldPosition, endPosition);
                context.cameFrom[startIndex] = -1;
                context.heapIndex[startIndex] = notInOpenSet;
                HeapPush(context, startIndex);

                int expansions = 0;

                while(context.heapCount > 0) {
                    if(!isOn) {
                        break;
                    }

                    int currentIndex = HeapPopMin(context);
                    if(currentIndex == endIndex) {
                        ReconstructPath(context, currentIndex, grids, width, height, resultPath);
                        return PathfindingStatus.Finished;
                    }

                    int currentX = currentIndex % width;
                    int currentY = (currentIndex / width) % height;
                    int currentZ = currentIndex / (width * height);
                    float currentGScore = context.gScore[currentIndex];

                    for(int i = 0; i < offsets.Length; i++) {
                        int neighbourX = currentX + offsets[i].x;
                        int neighbourY = currentY + offsets[i].y;
                        int neighbourZ = currentZ + offsets[i].z;

                        if(neighbourX < 0 || neighbourX >= width ||
                            neighbourY < 0 || neighbourY >= height ||
                            neighbourZ < 0 || neighbourZ >= length) {
                            continue;
                        }

                        GridNode neighbour = grids[neighbourX][neighbourY][neighbourZ];
                        if(neighbour.invalid) {
                            continue;
                        }

                        // No corner cutting: a diagonal step is only passable when the
                        // cells it squeezes past are also free (they lie between two
                        // in-bounds cells, so no bounds check is needed).
                        bool passable = true;
                        Vector3Int[] required = intermediates[i];
                        for(int r = 0; r < required.Length; r++) {
                            if(grids[currentX + required[r].x][currentY + required[r].y][currentZ + required[r].z].invalid) {
                                passable = false;
                                break;
                            }
                        }

                        if(!passable) {
                            continue;
                        }

                        int neighbourIndex = neighbourX + neighbourY * width + neighbourZ * width * height;

                        if(context.searchStamp[neighbourIndex] != context.searchId) {
                            context.searchStamp[neighbourIndex] = context.searchId;
                            context.gScore[neighbourIndex] = Mathf.Infinity;
                            context.heapIndex[neighbourIndex] = notInOpenSet;
                        }
                        else if(context.heapIndex[neighbourIndex] == closedNode) {
                            // No reopening: with a weighted heuristic the path stays within
                            // the weight bound and every node is expanded at most once.
                            continue;
                        }

                        float tentativeScore = currentGScore + pointDistance * costFactors[i];
                        if(tentativeScore < context.gScore[neighbourIndex]) {
                            context.cameFrom[neighbourIndex] = currentIndex;
                            context.gScore[neighbourIndex] = tentativeScore;
                            context.fScore[neighbourIndex] = tentativeScore + heuristicWeight * Vector3.Distance(neighbour.worldPosition, endPosition);

                            if(context.heapIndex[neighbourIndex] == notInOpenSet) {
                                HeapPush(context, neighbourIndex);
                            }
                            else {
                                HeapSiftUp(context, context.heapIndex[neighbourIndex]);
                            }
                        }
                    }

                    expansions++;
                    if(timeSliced && expansions >= maxExpansionsPerFrame) {
                        bool isCanceled = await UniTask.NextFrame(ct).SuppressCancellationThrow();

                        if(isCanceled) {
                            return PathfindingStatus.Invalid;
                        }

                        expansions = 0;
                    }
                }

                return PathfindingStatus.Invalid;
            }
            finally {
                context.busy = false;
            }
        }

        private static void ReconstructPath(SearchContext context, int endIndex, GridNode[][][] grids, int width, int height, List<GridNode> resultPath) {
            int safety = width * height * grids[0][0].Length; // total cell count, guards against cameFrom cycles
            int current = endIndex;

            while(current != -1 && safety > 0) {
                safety--;
                int x = current % width;
                int y = (current / width) % height;
                int z = current / (width * height);
                resultPath.Add(grids[x][y][z]);
                current = context.cameFrom[current];
            }
        }

        private List<Vector3> pruneCache = new List<Vector3>();

        private void PrunePath(List<Vector3> pathPoints, GridCollection collection) {
            if(!prunePathWaypoints || collection == null || pathPoints.Count < 3) {
                return;
            }

            pruneCache.Clear();
            pruneCache.Add(pathPoints[0]);
            int anchor = 0;

            for(int i = 2; i < pathPoints.Count; i++) {
                bool segmentTooLong = (pathPoints[i] - pathPoints[anchor]).magnitude > maxPrunedSegmentLength;

                if(segmentTooLong || !collection.HasLineOfSight(pathPoints[anchor], pathPoints[i])) {
                    // Keep the corner and its predecessor: the short knot entering the
                    // corner keeps follower smoothing small where clearance is tight.
                    if(!segmentTooLong && i - 2 > anchor) {
                        pruneCache.Add(pathPoints[i - 2]);
                    }

                    pruneCache.Add(pathPoints[i - 1]);
                    anchor = i - 1;
                }
            }

            pruneCache.Add(pathPoints[pathPoints.Count - 1]);
            pathPoints.Clear();
            pathPoints.AddRange(pruneCache);
        }

        #region Open set heap (indexed binary min-heap on fScore)
        private static void HeapPush(SearchContext context, int node) {
            int index = context.heapCount;
            context.heapCount++;
            context.heap[index] = node;
            context.heapIndex[node] = index;
            HeapSiftUp(context, index);
        }

        private static int HeapPopMin(SearchContext context) {
            int root = context.heap[0];
            context.heapIndex[root] = closedNode;
            context.heapCount--;

            if(context.heapCount > 0) {
                int last = context.heap[context.heapCount];
                context.heap[0] = last;
                context.heapIndex[last] = 0;
                HeapSiftDown(context, 0);
            }

            return root;
        }

        private static void HeapSiftUp(SearchContext context, int index) {
            int node = context.heap[index];
            float score = context.fScore[node];

            while(index > 0) {
                int parentIndex = (index - 1) / 2;
                int parentNode = context.heap[parentIndex];

                if(score >= context.fScore[parentNode]) {
                    break;
                }

                context.heap[index] = parentNode;
                context.heapIndex[parentNode] = index;
                index = parentIndex;
            }

            context.heap[index] = node;
            context.heapIndex[node] = index;
        }

        private static void HeapSiftDown(SearchContext context, int index) {
            int node = context.heap[index];
            float score = context.fScore[node];

            while(true) {
                int left = 2 * index + 1;
                if(left >= context.heapCount) {
                    break;
                }

                int smallest = left;
                int right = left + 1;
                if(right < context.heapCount && context.fScore[context.heap[right]] < context.fScore[context.heap[left]]) {
                    smallest = right;
                }

                if(context.fScore[context.heap[smallest]] >= score) {
                    break;
                }

                context.heap[index] = context.heap[smallest];
                context.heapIndex[context.heap[index]] = index;
                index = smallest;
            }

            context.heap[index] = node;
            context.heapIndex[node] = index;
        }
        #endregion

        public void SetPathColor(List<Vector3> totalPath) {
            if(debug) {
                if(totalPath != null) {
                    for(int j = totalPath.Count - 2; j >= 0; j--) {
                        Debug.DrawLine(totalPath[j + 1], totalPath[j], Color.blue, 2f);
                    }
                }
            }
        }
    }
}
