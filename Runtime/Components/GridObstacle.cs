using UnityEngine;
using System.Collections.Generic;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Dynamic blocker. The transform defines an oriented box (unit cube scaled by
    /// lossyScale); covered cells get refcounted dynamic blocks so overlapping
    /// obstacles and baked geometry are never corrupted by a release.
    /// An object should block through ONE channel: baked layers (never moves) or
    /// this component (dynamic) - not both.
    /// </summary>
    public class GridObstacle : MonoBehaviour {
        private static List<GridObstacle> obstacles = new ();

        [SerializeField, Min(0.3f)] private float obstacleUpdate = 0.5f;

        private float updateTime;
        private List<GridNode> coveredGrids = new List<GridNode>();
        private Vector3 lastLocation = Vector3.zero;
        private Quaternion lastRotation = Quaternion.identity;
        private int lastGridsVersion = -1;

        private void OnEnable() {
            obstacles.Add(this);
        }

        private void OnDisable() {
            UpdateCoveredGrid(false); //Release from block
            if(coveredGrids.Count > 0) {
                GridLocator.BumpObstacleVersion();
            }
            coveredGrids.Clear();
            obstacles.Remove(this);
        }

        /// <summary>
        /// Drive once per frame from the host game loop, or leave PathfindingTicker
        /// enabled to have it driven automatically.
        /// </summary>
        public static void TickAll() {
            var count = obstacles.Count;
            for(var i = 0; i < count; i++) {
                var obstacle = obstacles[i];
                obstacle.Tick();
            }

            GridLocator.TickIslandMaintenance();
        }

        private void Tick() {
            var time = Time.time;
            if(time - updateTime > obstacleUpdate) {
                UpdateGrid();
                updateTime = time;
            }
        }

        private void UpdateGrid() {
            if(transform.position != lastLocation ||
                transform.rotation != lastRotation ||
                coveredGrids.Count == 0 ||
                lastGridsVersion != GridLocator.activeGridsVersion) {
                NewUpdate();
            }
        }

        private void NewUpdate() {
            bool hadCells = coveredGrids.Count > 0;
            UpdateCoveredGrid(false); //Release from block
            coveredGrids.Clear();
            var count = GridLocator.activeGrid.Count;
            for(var i = 0; i < count; i++) {
                var manager = GridLocator.activeGrid[i];
                manager.GetCoveredGrid(transform, ref coveredGrids);
            }
            UpdateCoveredGrid(true); //Block the grid
            lastLocation = transform.position;
            lastRotation = transform.rotation;
            lastGridsVersion = GridLocator.activeGridsVersion;

            // Only a real coverage change invalidates island maps. An obstacle outside
            // every active grid volume (empty -> empty) must not churn the version,
            // or the dynamic island map could never become fresh.
            if(hadCells || coveredGrids.Count > 0) {
                GridLocator.BumpObstacleVersion();
            }
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="flag">Blocked?</param>
        private void UpdateCoveredGrid(bool flag) {
            for(int i = 0; i < coveredGrids.Count; i++) {
                if(coveredGrids[i] == null) {
                    continue;
                }

                if(flag) {
                    coveredGrids[i].AddDynamicBlock();
                }
                else {
                    coveredGrids[i].RemoveDynamicBlock();
                }
            }
        }

        private void OnDrawGizmos() {
            Gizmos.color = Color.blue;
            Matrix4x4 rotationMatrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale);
            Gizmos.matrix = rotationMatrix;
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
        }
    }
}
