using System.Collections.Generic;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    public class GridCollection : System.IDisposable {
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

        internal Vector3 startPoint;
        internal int gridWidth = 0;
        internal int gridHeight = 0;
        internal int gridLength = 0;
        internal float pointDistance;
        internal float sqrDistanceValidity;

        internal LayerMask layerMask;
        internal PathAgentType agentType;
        internal PathMovement movement;
        internal WalkSettings walk;
        internal bool invalidGridCollection { get; private set; }
        internal bool invalidGridObstacles { get; private set; }

        public Vector3 centerVisual;

        public int Width => gridWidth;
        public int Height => gridHeight;
        public int Length => gridLength;
        public float PointDistance => pointDistance;
        public Vector3 Origin => startPoint;
        public PathAgentType AgentType => agentType;
        /// <summary>How this grid's agents move through it.</summary>
        public PathMovement Movement => movement;
        /// <summary>What this grid's walkers' bodies allow (used when <see cref="Movement"/> is <see cref="PathMovement.Walk"/>).</summary>
        public WalkSettings Walk => walk;
        public bool IsWalking => movement == PathMovement.Walk;

        /// <summary>A flying grid.</summary>
        public GridCollection(int width, int height, int Length, float pointDistance, Vector3 centerPoint, PathAgentType agentType, LayerMask layermask)
            : this(width, height, Length, pointDistance, centerPoint, agentType, layermask, PathMovement.Fly, WalkSettings.Default) {
        }

        public GridCollection(int width, int height, int Length, float pointDistance, Vector3 centerPoint, PathAgentType agentType, LayerMask layermask, PathMovement movement, WalkSettings walk) {
            this.movement = movement;
            this.walk = walk;
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

        private GridSearchData searchData;
        private GridIslands islands;

        /// <summary>Native data searches run on (created on first search; released by <see cref="Dispose"/>).</summary>
        internal GridSearchData SearchData => searchData ??= new GridSearchData(this);

        /// <summary>
        /// The island maps: static (baked blocking only) and dynamic (obstacles too, refreshed in the background by
        /// GridLocator.TickIslandMaintenance and trusted only while fresh). Released by <see cref="Dispose"/>.
        /// </summary>
        internal GridIslands Islands => islands ??= new GridIslands(this);

        /// <summary>
        /// Releases the native search data and island maps (the grid stays usable: search data is rebuilt on the next
        /// search, the island maps by the next <see cref="GridExtension.ComputeIslands"/>).
        /// </summary>
        public void Dispose() {
            searchData?.Dispose();
            searchData = null;
            islands?.Dispose();
            islands = null;
        }

        public void InitializeGrid(GridNode[][][] grids) {
            this.grids = grids;
            searchData?.Invalidate();
            invalidGridCollection = grids == null;
            invalidGridObstacles = invalidGridCollection;
        }

        public void InitializeComplete() {
            searchData?.Invalidate();
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
            return islands != null && islands.HasFreshDynamic;
        }

        /// <summary>
        /// O(1) reachability pre-check. Uses the combined static+dynamic island map when
        /// it is fresh (exact both ways); otherwise falls back to the static-only map,
        /// which is conservative: it never rejects a reachable goal.
        /// </summary>
        public bool AreConnected(GridNode a, GridNode b) {
            if(HasFreshDynamicIslands()) {
                return islands.DynamicIsland(FlattenIndex(a.coords)) == islands.DynamicIsland(FlattenIndex(b.coords));
            }

            if(islands == null || !islands.HasStatic) {
                return true;
            }

            return islands.StaticIsland(FlattenIndex(a.coords)) == islands.StaticIsland(FlattenIndex(b.coords));
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
            // The probe floods flying moves; on a walking grid it proves nothing, so the search decides.
            if(invalidGridCollection || IsWalking) {
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

        /// <summary>
        /// Walking: a walker could stand along a straight line between two cells it stands in. Every column the line
        /// crosses (in plan) has a cell to stand in within a step of the last one, its body fits through each move, and
        /// the line ends at the height of <paramref name="to"/>. No drops: a ledge breaks the line.
        /// </summary>
        public bool HasWalkableLineOfSight(Vector3 from, Vector3 to) {
            if(invalidGridCollection || !IsWalking) {
                return false;
            }

            Vector3 start = (from - startPoint) / pointDistance;
            Vector3 end = (to - startPoint) / pointDistance;
            int x = Mathf.RoundToInt(start.x);
            int y = Mathf.RoundToInt(start.y);
            int z = Mathf.RoundToInt(start.z);
            int endX = Mathf.RoundToInt(end.x);
            int endY = Mathf.RoundToInt(end.y);
            int endZ = Mathf.RoundToInt(end.z);
            if(!IsStandable(x, y, z)) {
                return false;
            }

            float deltaX = end.x - start.x;
            float deltaZ = end.z - start.z;
            int stepX = deltaX > 0f ? 1 : -1;
            int stepZ = deltaZ > 0f ? 1 : -1;
            float tMaxX = Mathf.Abs(deltaX) > 0.000001f ? (x + 0.5f * stepX - start.x) / deltaX : float.PositiveInfinity;
            float tMaxZ = Mathf.Abs(deltaZ) > 0.000001f ? (z + 0.5f * stepZ - start.z) / deltaZ : float.PositiveInfinity;
            float tDeltaX = Mathf.Abs(deltaX) > 0.000001f ? Mathf.Abs(1f / deltaX) : float.PositiveInfinity;
            float tDeltaZ = Mathf.Abs(deltaZ) > 0.000001f ? Mathf.Abs(1f / deltaZ) : float.PositiveInfinity;
            int climb = walk.StepCells(pointDistance);
            int clearance = walk.ClearanceCells(pointDistance);
            int safety = Mathf.Abs(endX - x) + Mathf.Abs(endZ - z) + 3;

            while(safety-- > 0) {
                if(x == endX && z == endZ) {
                    return y == endY;
                }

                int nextX = x;
                int nextZ = z;
                if(Mathf.Abs(tMaxX - tMaxZ) < 0.0001f) {
                    // Through a corner exactly: a diagonal move, as the search makes it (no squeezing past the corner).
                    nextX += stepX;
                    nextZ += stepZ;
                    tMaxX += tDeltaX;
                    tMaxZ += tDeltaZ;
                }
                else if(tMaxX < tMaxZ) {
                    nextX += stepX;
                    tMaxX += tDeltaX;
                }
                else {
                    nextZ += stepZ;
                    tMaxZ += tDeltaZ;
                }

                // The cell to stand in nearest the current height, within a step either way.
                int found = int.MinValue;
                for(int offset = 0; offset <= climb && found == int.MinValue; offset++) {
                    if(IsStandable(nextX, y + offset, nextZ) && BodyFits(x, y, z, nextX, y + offset, nextZ, clearance)) {
                        found = y + offset;
                    }
                    else if(offset > 0 && IsStandable(nextX, y - offset, nextZ) && BodyFits(x, y, z, nextX, y - offset, nextZ, clearance)) {
                        found = y - offset;
                    }
                }

                if(found == int.MinValue) {
                    return false;
                }

                x = nextX;
                y = found;
                z = nextZ;
            }

            return false;
        }

        /// <summary>Walking: a walker can stand in this cell (free, a blocked cell under it, its body height free above).</summary>
        public bool IsStandable(int x, int y, int z) {
            if(grids == null || y < 1 || x < 0 || x >= gridWidth || y >= gridHeight || z < 0 || z >= gridLength) {
                return false;
            }

            return grids[x][y - 1][z].invalid && ColumnFree(x, z, y, y + walk.ClearanceCells(pointDistance) - 1);
        }

        /// <summary>
        /// Walking: the cell a walker would stand in nearest <paramref name="position"/>, within <paramref name="radius"/>
        /// cells. Nearness is measured to the floor of each cell, so a point on the ground (a foot, a spot on the floor)
        /// finds the cell standing on it.
        /// </summary>
        public bool TryGetStandable(Vector3 position, int radius, out GridNode node) {
            node = null;
            if(invalidGridCollection || grids == null) {
                return false;
            }

            GetWorldPositionToGrid(position, out int cx, out int cy, out int cz);
            float best = float.PositiveInfinity;
            float floorOffset = pointDistance * 0.5f;
            for(int x = cx - radius; x <= cx + radius; x++) {
                for(int y = cy - radius; y <= cy + radius + 1; y++) {
                    for(int z = cz - radius; z <= cz + radius; z++) {
                        if(!IsStandable(x, y, z)) {
                            continue;
                        }

                        GridNode candidate = grids[x][y][z];
                        float distance = (candidate.worldPosition - Vector3.up * floorOffset - position).sqrMagnitude;
                        if(distance < best) {
                            best = distance;
                            node = candidate;
                        }
                    }
                }
            }

            return node != null;
        }

        private bool ColumnFree(int x, int z, int fromY, int toY) {
            if(x < 0 || x >= gridWidth || z < 0 || z >= gridLength) {
                return false;
            }

            for(int y = Mathf.Max(fromY, 0); y <= toY && y < gridHeight; y++) {
                if(grids[x][y][z].invalid) {
                    return false;
                }
            }

            return fromY >= 0;
        }

        /// <summary>The managed twin of <see cref="WalkMoves.BodyFits"/>.</summary>
        private bool BodyFits(int fromX, int fromY, int fromZ, int toX, int toY, int toZ, int clearance) {
            int top = Mathf.Max(fromY, toY) + clearance - 1;
            if(!ColumnFree(fromX, fromZ, fromY, top) || !ColumnFree(toX, toZ, toY, top)) {
                return false;
            }

            int dx = toX - fromX;
            int dz = toZ - fromZ;
            if(dx != 0 && dz != 0) {
                int low = Mathf.Min(fromY, toY);
                return ColumnFree(fromX + dx, fromZ, low, top) && ColumnFree(fromX, fromZ + dz, low, top);
            }

            return true;
        }

        public bool CheckValidityLocation(Vector3 position) {
            if(invalidGridCollection) {
                return false;
            }

            if(IsWalking) {
                return TryGetStandable(position, 1, out _);
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

            if(IsWalking && getClosestValid) {
                // Walking: valid is a cell a walker stands in.
                if(!TryGetStandable(position, searchRadius, out result)) {
                    result = new GridNode();
                    result.BlockedArea(true);
                }

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
                    int x = Random.Range(2, gridWidth - 2);
                    int y = Random.Range(2, gridHeight - 2);
                    int z = Random.Range(2, gridLength - 2);
                    GridNode randomGrid = grids[x][y][z];

                    if(IsWalking ? IsStandable(x, y, z) : !randomGrid.invalid) {
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
