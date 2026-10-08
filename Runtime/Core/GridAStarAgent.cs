using Cysharp.Threading.Tasks;
using System.Collections.Generic;
using System.Threading;
using Unity.Jobs;
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

        /// <summary>
        /// Asynchronous searches allowed to run at once across all agents (each runs as a Burst job on a worker thread).
        /// More wait their turn, in order, which keeps worker threads free for physics and rendering and bounds the
        /// scratch memory (one set per search in flight).
        /// </summary>
        public static int MaxConcurrentSearches = 2;

        private static int runningSearches;

        private GridCollection gridCollection;
        private bool isOn;
        private bool debug;

        private List<GridNode> tempPath = new List<GridNode>();
        private readonly List<GridNode> asyncPath = new List<GridNode>();
        private bool asyncPathInUse;

        public ReentryCanceller reentryCanceller = new();

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
            List<Vector3> pathPoints = new List<Vector3>();
            bool success = await GetPathAsync(targetLocation, pathPoints, ct);
            return (success, pathPoints);
        }

        /// <summary>
        /// Finds a path to <paramref name="targetLocation"/> into the caller's <paramref name="pathPoints"/> (cleared first),
        /// on a worker thread; nothing is allocated per search.
        /// </summary>
        public async UniTask<bool> GetPathAsync(Vector3 targetLocation, List<Vector3> pathPoints, CancellationToken ct = default) {
            // Captured before the awaits so pruning runs against the same collection the
            // search used, even if the agent is toggled or rebound mid-search.
            GridCollection collection = gridCollection;
            pathPoints.Clear();
            // One search per agent at a time owns the reusable node list; an overlapping one gets its own.
            List<GridNode> gridPath = asyncPathInUse ? new List<GridNode>() : asyncPath;
            bool ownsShared = gridPath == asyncPath;
            asyncPathInUse |= ownsShared;
            try {
                PathfindingStatus status = await PathfindingCore(agentTransform.position, targetLocation, gridPath, timeSliced: true, ct);
                if(status == PathfindingStatus.Invalid) {
                    return false;
                }

                for(int i = gridPath.Count - 1; i > 0; i--) {
                    pathPoints.Add(gridPath[i].worldPosition);
                }
            }
            finally {
                if(ownsShared) {
                    asyncPathInUse = false;
                }
            }

            PrunePath(pathPoints, collection);
            SetPathColor(pathPoints);
            return true;
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
            int endIndex = collection.FlattenIndex(end.coords);
            int startIndex = collection.FlattenIndex(start.coords);

            if(timeSliced) {
                // Searches past the concurrency cap wait their turn.
                while(runningSearches >= MaxConcurrentSearches) {
                    if(await UniTask.Yield(ct).SuppressCancellationThrow()) {
                        return PathfindingStatus.Invalid;
                    }
                }

                runningSearches++;
            }

            GridSearchData search = collection.SearchData;
            GridSearchData.Scratch scratch = null;
            try {
                search.Refresh();
                scratch = search.Rent();
                var job = new AStarJob {
                    blocked = search.Blocked,
                    width = width,
                    height = height,
                    length = length,
                    pointDistance = collection.pointDistance,
                    heuristicWeight = heuristicWeight,
                    start = startIndex,
                    goal = endIndex,
                    maxExpansions = width * height * length,
                    gScore = scratch.gScore,
                    fScore = scratch.fScore,
                    cameFrom = scratch.cameFrom,
                    stamp = scratch.stamp,
                    heapIndex = scratch.heapIndex,
                    heap = scratch.heap,
                    stampId = scratch.NextStamp(),
                    path = scratch.path,
                    result = scratch.result
                };

                if(timeSliced) {
                    JobHandle handle = job.Schedule();
                    search.AddReader(handle);
                    JobHandle.ScheduleBatchedJobs();
                    bool canceled = false;
                    while(!handle.IsCompleted && !canceled) {
                        canceled = await UniTask.Yield(ct).SuppressCancellationThrow();
                    }

                    handle.Complete();
                    if(canceled) {
                        return PathfindingStatus.Invalid;
                    }
                }
                else {
                    job.Run();
                }

                if(!isOn || search.IsDisposed || scratch.result[0] != AStarJob.Finished) {
                    return PathfindingStatus.Invalid;
                }

                // The job lists the path from the goal back to the start, as cell indices.
                for(int i = 0; i < scratch.path.Length; i++) {
                    int index = scratch.path[i];
                    resultPath.Add(grids[index % width][(index / width) % height][index / (width * height)]);
                }

                return PathfindingStatus.Finished;
            }
            finally {
                if(scratch != null) {
                    search.Return(scratch);
                }

                if(timeSliced) {
                    runningSearches--;
                }
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
