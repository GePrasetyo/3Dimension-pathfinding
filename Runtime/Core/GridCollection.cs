using System.Collections.Generic;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    public class GridCollection {
        // The 26 surrounding cells with their step cost relative to pointDistance (1, sqrt2, sqrt3).
        // Neighbours are implicit from coordinates: on a uniform grid they are fully
        // derivable, so nothing is stored per cell.
        internal static readonly Vector3Int[] neighbourOffsets;
        internal static readonly float[] neighbourCostFactors;

        // Cells that must also be free for a diagonal step to be physically passable
        // (no corner cutting): every sub-offset obtained by zeroing one or two axes.
        // Empty for single-axis steps.
        internal static readonly Vector3Int[][] neighbourIntermediates;

        static GridCollection() {
            neighbourOffsets = new Vector3Int[26];
            neighbourCostFactors = new float[26];
            neighbourIntermediates = new Vector3Int[26][];
            int index = 0;

            for(int x = -1; x <= 1; x++) {
                for(int y = -1; y <= 1; y++) {
                    for(int z = -1; z <= 1; z++) {
                        if(x == 0 && y == 0 && z == 0) {
                            continue;
                        }

                        neighbourOffsets[index] = new Vector3Int(x, y, z);
                        neighbourCostFactors[index] = Mathf.Sqrt(x * x + y * y + z * z);

                        List<Vector3Int> intermediates = new List<Vector3Int>();
                        for(int mask = 1; mask < 7; mask++) {
                            Vector3Int sub = new Vector3Int(
                                (mask & 1) != 0 ? x : 0,
                                (mask & 2) != 0 ? y : 0,
                                (mask & 4) != 0 ? z : 0);

                            if(sub != Vector3Int.zero && sub != neighbourOffsets[index] && !intermediates.Contains(sub)) {
                                intermediates.Add(sub);
                            }
                        }
                        neighbourIntermediates[index] = intermediates.ToArray();

                        index++;
                    }
                }
            }
        }

        internal GridNode[][][] grids;

        // Island id per cell (flattened), computed from static blocking only. 0 = blocked/unassigned.
        internal int[] islandIds;

        // Islands computed from combined blocking (static + dynamic obstacles), refreshed
        // by GridLocator.TickIslandMaintenance. Only trusted while the version stamp
        // matches GridLocator.obstacleVersion; stale maps fall back to islandIds.
        internal int[] dynamicIslandIds;
        internal int dynamicIslandVersion = -1;
        internal bool dynamicIslandFillRunning;
        internal float nextDynamicIslandTime;

        internal Vector3 startPoint;
        internal int gridWidth = 0;
        internal int gridHeight = 0;
        internal int gridLength = 0;
        internal float pointDistance;
        internal float sqrDistanceValidity;

        internal LayerMask layerMask;
        internal PathAgentType agentType;
        internal bool invalidGridCollection { get; private set; }
        internal bool invalidGridObstacles { get; private set; }

        public Vector3 centerVisual;

        public int Width => gridWidth;
        public int Height => gridHeight;
        public int Length => gridLength;
        public float PointDistance => pointDistance;
        public Vector3 Origin => startPoint;
        public PathAgentType AgentType => agentType;

        public GridCollection(int width, int height, int Length, float pointDistance, Vector3 centerPoint, PathAgentType agentType, LayerMask layermask) {
            this.gridWidth = width;
            this.gridHeight = height;
            this.gridLength = Length;

            this.pointDistance = pointDistance;
            this.sqrDistanceValidity = pointDistance * pointDistance;
            this.startPoint = new Vector3(-width, -height, -Length) / 2f * pointDistance + centerPoint;

            this.agentType = agentType;
            this.layerMask = layermask;
            this.invalidGridCollection = true;
            this.invalidGridObstacles = true;
            this.centerVisual = centerPoint;
        }

        public void InitializeGrid(GridNode[][][] grids) {
            this.grids = grids;
            invalidGridCollection = grids == null;
            invalidGridObstacles = invalidGridCollection;
        }

        public void InitializeComplete() {
            invalidGridCollection = false;
            invalidGridObstacles = false;
        }

        public void SetGridObstacleInvalid(bool value) {
            invalidGridObstacles = value;
        }

        public int FlattenIndex(Vector3Int coords) {
            return coords.x + coords.y * gridWidth + coords.z * gridWidth * gridHeight;
        }

        public bool HasFreshDynamicIslands() {
            return dynamicIslandIds != null &&
                dynamicIslandIds.Length == gridWidth * gridHeight * gridLength &&
                dynamicIslandVersion == GridLocator.obstacleVersion;
        }

        /// <summary>
        /// O(1) reachability pre-check. Uses the combined static+dynamic island map when
        /// it is fresh (exact both ways); otherwise falls back to the static-only map,
        /// which is conservative: it never rejects a reachable goal.
        /// </summary>
        public bool AreConnected(GridNode a, GridNode b) {
            if(HasFreshDynamicIslands()) {
                return dynamicIslandIds[FlattenIndex(a.coords)] == dynamicIslandIds[FlattenIndex(b.coords)];
            }

            if(islandIds == null) {
                return true;
            }

            return islandIds[FlattenIndex(a.coords)] == islandIds[FlattenIndex(b.coords)];
        }

        private static readonly Queue<Vector3Int> probeFrontier = new Queue<Vector3Int>();
        private static readonly HashSet<int> probeVisited = new HashSet<int>();

        /// <summary>
        /// Bounded reverse flood from the goal over live blocking (static + dynamic).
        /// Returns true only on PROOF that the goal sits in a sealed pocket that does
        /// not contain the start (frontier exhausted within budget). Reaching the start
        /// or running out of budget returns false, so this can never reject a reachable
        /// goal - it only fast-fails provably sealed ones while the island map is stale.
        /// </summary>
        public bool GoalPocketSealed(GridNode goal, GridNode start, int budget = 256) {
            if(invalidGridCollection) {
                return false;
            }

            int startIndex = FlattenIndex(start.coords);
            probeFrontier.Clear();
            probeVisited.Clear();
            probeVisited.Add(FlattenIndex(goal.coords));
            probeFrontier.Enqueue(goal.coords);

            int processed = 0;
            while(probeFrontier.Count > 0) {
                if(processed >= budget) {
                    return false; // inconclusive - let the full search decide
                }

                processed++;
                Vector3Int current = probeFrontier.Dequeue();

                for(int i = 0; i < neighbourOffsets.Length; i++) {
                    Vector3Int next = current + neighbourOffsets[i];

                    if(next.x < 0 || next.x >= gridWidth ||
                        next.y < 0 || next.y >= gridHeight ||
                        next.z < 0 || next.z >= gridLength) {
                        continue;
                    }

                    if(grids[next.x][next.y][next.z].invalid) {
                        continue;
                    }

                    int nextIndex = FlattenIndex(next);
                    if(nextIndex == startIndex) {
                        return false; // provably connected
                    }

                    if(probeVisited.Add(nextIndex)) {
                        probeFrontier.Enqueue(next);
                    }
                }
            }

            // Frontier exhausted: the goal pocket is fully mapped and does not contain the start.
            return true;
        }

        /// <summary>
        /// Exact voxel traversal (Amanatides-Woo DDA) of the segment against grid
        /// validity (static and dynamic blocking): every cell the segment crosses is
        /// checked, so corner-grazing lines cannot slip between samples.
        /// Endpoints are assumed already validated by the caller.
        /// </summary>
        public bool HasLineOfSight(Vector3 from, Vector3 to) {
            if(invalidGridCollection) {
                return false;
            }

            // Cell space: cell i is centered on integer i, boundaries at half-integers.
            Vector3 start = (from - startPoint) / pointDistance;
            Vector3 end = (to - startPoint) / pointDistance;
            Vector3 delta = end - start;

            int x = Mathf.RoundToInt(start.x);
            int y = Mathf.RoundToInt(start.y);
            int z = Mathf.RoundToInt(start.z);
            int endX = Mathf.RoundToInt(end.x);
            int endY = Mathf.RoundToInt(end.y);
            int endZ = Mathf.RoundToInt(end.z);

            int stepX = delta.x > 0f ? 1 : -1;
            int stepY = delta.y > 0f ? 1 : -1;
            int stepZ = delta.z > 0f ? 1 : -1;

            // t (0..1 along the segment) at which the ray crosses the next cell boundary
            // per axis, and how much t one full cell costs per axis.
            float tMaxX = Mathf.Abs(delta.x) > 0.000001f ? (x + 0.5f * stepX - start.x) / delta.x : float.PositiveInfinity;
            float tMaxY = Mathf.Abs(delta.y) > 0.000001f ? (y + 0.5f * stepY - start.y) / delta.y : float.PositiveInfinity;
            float tMaxZ = Mathf.Abs(delta.z) > 0.000001f ? (z + 0.5f * stepZ - start.z) / delta.z : float.PositiveInfinity;
            float tDeltaX = Mathf.Abs(delta.x) > 0.000001f ? Mathf.Abs(1f / delta.x) : float.PositiveInfinity;
            float tDeltaY = Mathf.Abs(delta.y) > 0.000001f ? Mathf.Abs(1f / delta.y) : float.PositiveInfinity;
            float tDeltaZ = Mathf.Abs(delta.z) > 0.000001f ? Mathf.Abs(1f / delta.z) : float.PositiveInfinity;

            int safety = Mathf.Abs(endX - x) + Mathf.Abs(endY - y) + Mathf.Abs(endZ - z) + 3;

            while(safety-- > 0) {
                if(x < 0 || x >= gridWidth || y < 0 || y >= gridHeight || z < 0 || z >= gridLength) {
                    return false;
                }

                if(grids[x][y][z].invalid) {
                    return false;
                }

                if(x == endX && y == endY && z == endZ) {
                    return true;
                }

                if(tMaxX <= tMaxY && tMaxX <= tMaxZ) {
                    x += stepX;
                    tMaxX += tDeltaX;
                }
                else if(tMaxY <= tMaxZ) {
                    y += stepY;
                    tMaxY += tDeltaY;
                }
                else {
                    z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }

            return false;
        }

        public bool CheckValidityLocation(Vector3 position) {
            if(invalidGridCollection) {
                return false;
            }

            GetWorldPositionToGrid(position, out int x, out int y, out int z);
            float distanceCheck = (grids[x][y][z].worldPosition - position).sqrMagnitude;

            return distanceCheck < sqrDistanceValidity && !grids[x][y][z].invalid;
        }

        /// <summary>
        /// Collects all cells covered by the obstacle's oriented unit-cube bounds
        /// (transform scale defines the box size).
        /// </summary>
        /// <param name="closestGrids">Don't pass null list</param>
        public void GetCoveredGrid(Transform obstacle, ref List<GridNode> closestGrids) {
            if(invalidGridCollection) {
                return;
            }

            Matrix4x4 matrix = Matrix4x4.TRS(obstacle.position, obstacle.rotation, obstacle.lossyScale).inverse;
            GetWorldPositionToGrid(obstacle.position, out int x, out int y, out int z);
            GridNode closestGrid = grids[x][y][z];

            float biggestScale = Mathf.Max(obstacle.lossyScale.z, obstacle.lossyScale.y, obstacle.lossyScale.x);
            // Calculate the number of rows, columns, and layers of grids that are covered by the object
            int rows = Mathf.CeilToInt(biggestScale / pointDistance);
            int columns = Mathf.CeilToInt(biggestScale / pointDistance);
            int layers = Mathf.CeilToInt(biggestScale / pointDistance);
            Vector3 point = Vector3.zero;

            // Loop through the multi-dimensional array of grids to find all the grids that are within the specified rows, columns, and layers
            for(int i = closestGrid.coords.x - columns / 2; i <= closestGrid.coords.x + columns / 2; i++) {
                for(int j = closestGrid.coords.y - layers / 2; j <= closestGrid.coords.y + layers / 2; j++) {
                    for(int k = closestGrid.coords.z - rows / 2; k <= closestGrid.coords.z + rows / 2; k++) {
                        if(i >= 0 && i < gridWidth && j >= 0 && j < gridHeight && k >= 0 && k < gridLength) {
                            point = matrix.MultiplyPoint3x4(grids[i][j][k].worldPosition);

                            if(Mathf.Abs(point.x) <= 0.5f && Mathf.Abs(point.y) <= 0.5f && Mathf.Abs(point.z) <= 0.5f) {
                                closestGrids.Add(grids[i][j][k]);
                            }
                        }
                    }
                }
            }
        }

        public void GetClosestPointWorldSpace(Vector3 position, out GridNode result, bool getClosestValid = true, int searchRadius = 1) {
            if(invalidGridCollection) {
                result = new GridNode();
                result.BlockedArea(true);
                return;
            }

            GetWorldPositionToGrid(position, out int x, out int y, out int z);
            result = grids[x][y][z];

            if(!getClosestValid) {
                return;
            }

            float distance = Mathf.Infinity;

            for(int p = -searchRadius; p <= searchRadius; p++) {
                for(int q = -searchRadius; q <= searchRadius; q++) {
                    for(int g = -searchRadius; g <= searchRadius; g++) {
                        int i = x + p;
                        int j = y + q;
                        int k = z + g;

                        // Check if within grid bounds and valid
                        if(i > -1 && i < gridWidth && j > -1 && j < gridHeight && k > -1 && k < gridLength && !grids[i][j][k].invalid) {
                            float dist = (grids[i][j][k].worldPosition - position).sqrMagnitude;
                            if(dist < distance) {
                                result = grids[i][j][k];
                                distance = dist;
                            }
                        }
                    }
                }
            }
        }

        private void GetWorldPositionToGrid(Vector3 position, out int x, out int y, out int z) {
            float sizeX = pointDistance * gridWidth;
            float sizeY = pointDistance * gridHeight;
            float sizeZ = pointDistance * gridLength;

            Vector3 pos = position - startPoint;
            float percentageX = Mathf.Clamp01(pos.x / sizeX);
            float percentageY = Mathf.Clamp01(pos.y / sizeY);
            float percentageZ = Mathf.Clamp01(pos.z / sizeZ);
            x = Mathf.Clamp(Mathf.RoundToInt(percentageX * gridWidth), 0, gridWidth - 1);
            y = Mathf.Clamp(Mathf.RoundToInt(percentageY * gridHeight), 0, gridHeight - 1);
            z = Mathf.Clamp(Mathf.RoundToInt(percentageZ * gridLength), 0, gridLength - 1);
        }

        public bool GetRandomPointInGrid(out Vector3 point, int totalAttempt = 16) {
            if(!invalidGridCollection) {
                for(int i = 0; i < totalAttempt; i++) {
                    GridNode randomGrid = grids[Random.Range(2, gridWidth - 2)][Random.Range(2, gridHeight - 2)][Random.Range(2, gridLength - 2)];

                    if(!randomGrid.invalid) {
                        point = randomGrid.worldPosition;
                        return true;
                    }
                }
            }

            point = Vector3.zero;
            return false;
        }
    }
}
