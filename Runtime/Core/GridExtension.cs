using Cysharp.Threading.Tasks;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    public static class GridExtension {
        private const float maxFrameTime = 0.01f;

        internal static bool ProbeBlocked(GridCollection data, Vector3 position) {
            return Physics.CheckBox(position, Vector3.one * ((data.pointDistance / 2f) + 0.1f), Quaternion.identity, data.layerMask);
        }

        /// <summary>
        /// Editor-bake sampler: builds the full node array with static blocking from
        /// physics, without touching the collection. The result is meant to be
        /// serialized via GridDataSerializer.
        /// </summary>
        public static async UniTask<GridNode[][][]> GenerateGrid(this GridCollection data) {
            GridNode[][][] bakedGrids = new GridNode[data.gridWidth][][];
            int splitOptimizationA = Mathf.Max(1, (data.gridHeight / 2));
            int splitOptimizationB = Mathf.Max(1, (data.gridWidth / 2));
            for(int x = 0; x < data.gridWidth; x++) {
                bakedGrids[x] = new GridNode[data.gridHeight][];

                for(int y = 0; y < data.gridHeight; y++) {
                    bakedGrids[x][y] = new GridNode[data.gridLength];

                    for(int z = 0; z < data.gridLength; z++) {
                        Vector3 pos = data.startPoint + new Vector3(x, y, z) * data.pointDistance;
                        bakedGrids[x][y][z] = new GridNode();
                        bakedGrids[x][y][z].coords = new Vector3Int(x, y, z);
                        bakedGrids[x][y][z].worldPosition = pos;
                        bakedGrids[x][y][z].BlockedArea(ProbeBlocked(data, pos));
                    }
                    if(y % splitOptimizationA == 0) {
                        await UniTask.Delay(20);
                    }
                }
                if(x % splitOptimizationB == 0) {
                    await UniTask.Delay(20);
                }
            }

            return bakedGrids;
        }

        /// <summary>
        /// Runtime grid build: samples static blocking from physics, time-sliced so
        /// large grids never stall a frame, then computes islands.
        /// </summary>
        public static async UniTask GenerateGridAsync(this GridCollection data) {
            float taskTime = Time.realtimeSinceStartup;

            data.grids = new GridNode[data.gridWidth][][];
            for(int x = 0; x < data.gridWidth; x++) {
                data.grids[x] = new GridNode[data.gridHeight][];
                for(int y = 0; y < data.gridHeight; y++) {
                    data.grids[x][y] = new GridNode[data.gridLength];
                    for(int z = 0; z < data.gridLength; z++) {
                        GridNode grid = new GridNode();
                        grid.coords = new Vector3Int(x, y, z);
                        grid.worldPosition = data.startPoint + new Vector3(x, y, z) * data.pointDistance;
                        grid.BlockedArea(ProbeBlocked(data, grid.worldPosition));
                        data.grids[x][y][z] = grid;

                        if(Time.realtimeSinceStartup - taskTime > maxFrameTime) {
                            await UniTask.NextFrame();
                            taskTime = Time.realtimeSinceStartup;
                        }
                    }
                }
            }

            data.InitializeComplete();
            await data.ComputeIslands();
        }

        /// <summary>
        /// Re-samples static blocking from physics over the whole grid, then rebuilds
        /// islands. Use after level geometry changes for a reused collection.
        /// </summary>
        public static async UniTask RevalidateGridPath(this GridCollection data) {
            GridNode grid;
            float taskTime = Time.realtimeSinceStartup;

            for(int x = 0; x < data.gridWidth; x++) {
                for(int y = 0; y < data.gridHeight; y++) {
                    for(int z = 0; z < data.gridLength; z++) {
                        grid = data.grids[x][y][z];
                        grid.BlockedArea(ProbeBlocked(data, grid.worldPosition));

                        if(Time.realtimeSinceStartup - taskTime > maxFrameTime) {
                            await UniTask.NextFrame();
                            taskTime = Time.realtimeSinceStartup;
                        }
                    }
                }
            }

            await data.ComputeIslands();
            data.SetGridObstacleInvalid(false);
        }

        /// <summary>
        /// Flood fills statically walkable cells into connected regions so unreachable
        /// path requests fail in O(1) instead of exhausting the whole grid.
        /// Uses static blocking only, which keeps the fallback check conservative
        /// (never rejects a reachable goal). Also invalidates the dynamic island map,
        /// since it was built on top of the previous static data. The fill is a Burst
        /// job, complete when this returns.
        /// </summary>
        public static UniTask ComputeIslands(this GridCollection data) {
            if(data.grids == null) {
                return UniTask.CompletedTask;
            }

            data.Islands.ComputeStatic();

            // Static data changed: invalidate any dynamic fill that started before this
            // point, or it would complete stamped as fresh over mixed old/new data.
            GridLocator.BumpObstacleVersion();
            return UniTask.CompletedTask;
        }

        /// <summary>
        /// Combined static+dynamic island map, filled now and stamped with the current
        /// obstacle version. GridLocator.TickIslandMaintenance keeps it fresh in the
        /// background on its own; call this only when a fresh map is needed at once.
        /// </summary>
        public static UniTask ComputeDynamicIslands(this GridCollection data) {
            if(data.grids != null) {
                data.Islands.ComputeDynamic();
            }

            return UniTask.CompletedTask;
        }
    }
}
