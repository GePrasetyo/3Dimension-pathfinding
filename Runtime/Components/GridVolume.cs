using Cysharp.Threading.Tasks;
using UnityEngine;
using UnityEngine.AddressableAssets;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Defines one navigable grid volume for an agent type. At runtime the host calls
    /// InitializeGrid() during level load (baked data or runtime generation) and
    /// Activate() to make this the live grid for its agent type. Baking happens
    /// through the inspector button provided by the editor assembly.
    /// </summary>
    public class GridVolume : MonoBehaviour {
        [SerializeField] private PathAgentType agentType;
        [SerializeField] private int gridWidth = 0;
        [SerializeField] private int gridHeight = 0;
        [SerializeField] private int gridLength = 0;
        [SerializeField] private float pointDistance = 0.5f;
        [SerializeField] private LayerMask layermask;
        [SerializeField] private AssetReference gridData;

        [Header("Do not enable this unless it's a testing/sandbox level!")]
        [SerializeField] private bool generateAtRuntime;

        [Header("Editor Bake")]
        [SerializeField] private string bakedOutputFolder = "Assets/PathfindingData";

        [SerializeField] private bool debugObstacle;
        [SerializeField] private bool debugWalkable;

        private GridCollection gridCollection;

        public PathAgentType AgentType => agentType;
        public LayerMask BakeLayerMask => layermask;
        public string BakedOutputFolder => bakedOutputFolder;

        /// <summary>
        /// Builds a fresh collection from the serialized volume settings. Used by
        /// Awake, runtime generation and the editor baker, so the configuration
        /// lives in exactly one place.
        /// </summary>
        public GridCollection BuildCollection() {
            return new GridCollection(
                gridWidth,
                gridHeight,
                gridLength,
                pointDistance,
                transform.position,
                agentType,
                layermask);
        }

        private void Awake() {
            gridCollection = BuildCollection();
            GridLocator.Add(gridCollection);
            Activate();
        }

        private void OnDestroy() {
            GridLocator.Remove(gridCollection);
        }

        public void Activate() {
            if(gridCollection != null) {
                GridLocator.Activate(gridCollection);
            }
        }

        public async UniTask<bool> InitializeGrid() {
            if(generateAtRuntime) {
                await gridCollection.GenerateGridAsync();
            }
            else {
                bool hasData = gridData != null && gridData.RuntimeKeyIsValid();
                gridCollection.InitializeGrid(hasData ? await GridDataSerializer.LoadGridData(gridData, gridCollection) : null);
            }

            if(gridCollection.grids == null) {
                return false;
            }

            // GenerateGridAsync computes islands itself; the baked path needs it here.
            if(!generateAtRuntime) {
                await gridCollection.ComputeIslands();
            }

            return true;
        }

        private void OnDrawGizmos() {
            Gizmos.DrawWireCube(transform.position, new Vector3(gridWidth, gridHeight, gridLength) * pointDistance);

            if(gridCollection?.grids == null || gridCollection.invalidGridCollection) return;

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
                                Gizmos.DrawWireSphere(gridCollection.grids[g][c][b].worldPosition, pointDistance / 4f);
                            }

                            continue;
                        }

                        if(debugWalkable) {
                            Gizmos.color = Color.white;
                            Gizmos.DrawWireCube(gridCollection.grids[g][c][b].worldPosition, Vector3.one * pointDistance);
                        }
                    }
                }
            }
        }

        [ContextMenu("Build World")]
        public void DebugInitializeGrid() {
            gridCollection = BuildCollection();
            gridCollection.GenerateGridAsync().Forget();
        }
    }
}
