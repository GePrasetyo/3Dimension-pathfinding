using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Debug visualization for a runtime-created GridCollection (grids created in
    /// code rather than via a GridVolume component).
    /// </summary>
    public class GridVisualizeGizmo : MonoBehaviour {
        public GridCollection gridCollection;
        [SerializeField] private bool debugObstacle;
        [SerializeField] private bool debugWalkable;

        public void Setup(GridCollection grid) {
            gridCollection = grid;

            transform.position = grid.centerVisual;
            transform.rotation = Quaternion.identity;
        }

        private void OnDrawGizmos() {
            if(gridCollection?.grids == null || gridCollection.invalidGridCollection) return;

            Gizmos.DrawWireCube(transform.position, new Vector3(gridCollection.gridWidth, gridCollection.gridHeight, gridCollection.gridLength) * gridCollection.pointDistance);

            if(!debugObstacle && !debugWalkable)
                return;

            for(int g = 0; g < gridCollection.grids.Length; g++) {
                for(int c = 0; c < gridCollection.grids[g].Length; c++) {
                    if(gridCollection.grids[g][c] == null) {
                        Debug.LogError($"Grid Null {g} {c}");
                        continue;
                    }

                    for(int b = 0; b < gridCollection.grids[g][c].Length; b++) {
                        if(gridCollection.grids[g][c][b] == null) {
                            Debug.LogError($"Grid Null {g} {c} {b}");
                            continue;
                        }

                        if(gridCollection.grids[g][c][b].invalid) {
                            if(debugObstacle) {
                                Gizmos.color = Color.red;
                                Gizmos.DrawWireSphere(gridCollection.grids[g][c][b].worldPosition, gridCollection.pointDistance / 4f);
                            }

                            continue;
                        }

                        if(debugWalkable) {
                            Gizmos.color = Color.white;
                            Gizmos.DrawWireCube(gridCollection.grids[g][c][b].worldPosition, Vector3.one * gridCollection.pointDistance);
                        }
                    }
                }
            }
        }
    }
}
