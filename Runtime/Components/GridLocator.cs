using Cysharp.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    public static class GridLocator {
        internal static Dictionary<PathAgentType, GridCollection> activeGridByType = new Dictionary<PathAgentType, GridCollection>();
        internal static List<GridCollection> activeGrid = new List<GridCollection>();
        internal static List<GridCollection> gridCollections = new List<GridCollection>();

        // Bumped on every dynamic obstacle change; island maps stamped with an older
        // version are ignored (queries fall back to the static-only map).
        internal static int obstacleVersion;

        // Bumped when the active grid set changes so stationary obstacles re-cover
        // newly activated or rebuilt collections.
        internal static int activeGridsVersion;

        private const float dynamicIslandCooldown = 1f;

        public static void BumpObstacleVersion() {
            obstacleVersion++;
        }

        public static GridCollection GetActive(PathAgentType type) {
            if(type != null && activeGridByType.TryGetValue(type, out GridCollection grid)) {
                return grid;
            }

            return null;
        }

        public static void Activate(GridCollection gridCollection) {
            if(activeGridByType.TryGetValue(gridCollection.agentType, out var manager)) {
                activeGrid.Remove(manager);
            }

            activeGrid.Add(gridCollection);
            activeGridByType[gridCollection.agentType] = gridCollection;
            activeGridsVersion++;
        }

        public static void Add(GridCollection gridCollection) {
            gridCollections.Add(gridCollection);
        }

        public static void Remove(GridCollection gridCollection) {
            if(activeGridByType.TryGetValue(gridCollection.agentType, out var manager)) {
                if(manager == gridCollection) {
                    activeGridByType.Remove(gridCollection.agentType);
                    activeGrid.Remove(gridCollection);
                }
            }

            gridCollections.Remove(gridCollection);
            // Its native search data goes with it.
            gridCollection.Dispose();
        }

        public static bool IsPathValid(PathAgentType agent, Vector3 point) {
            GridCollection gridCollection = GetActive(agent);
            if(gridCollection == null) {
                return false;
            }

            if(!gridCollection.CheckValidityLocation(point)) {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Coalesced dynamic island refresh: at most one time-sliced flood fill in
        /// flight per collection, started at most once per cooldown, and only when
        /// the current map is stale. Many obstacle changes collapse into one refill.
        /// </summary>
        public static void TickIslandMaintenance() {
            for(int i = 0; i < activeGrid.Count; i++) {
                GridCollection grid = activeGrid[i];

                if(grid.invalidGridCollection || grid.grids == null) {
                    continue;
                }

                if(grid.dynamicIslandFillRunning || grid.HasFreshDynamicIslands()) {
                    continue;
                }

                if(Time.time < grid.nextDynamicIslandTime) {
                    continue;
                }

                grid.nextDynamicIslandTime = Time.time + dynamicIslandCooldown;
                grid.ComputeDynamicIslands().Forget();
            }
        }
    }
}
