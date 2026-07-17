using System.Threading.Tasks;
using UnityEditor;
using UnityEditor.AddressableAssets;
using UnityEditor.AddressableAssets.Settings;
using UnityEngine;

namespace Majinfwork.Pathfinding.Editor {
    /// <summary>
    /// Bakes a GridVolume's static blocking into a PathfindingGridData asset,
    /// registers it with Addressables and assigns the AssetReference back to the
    /// volume.
    /// </summary>
    public static class GridVolumeBaker {
        public static async Task BakeAllInScene() {
            GridVolume[] volumes = Object.FindObjectsByType<GridVolume>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            try {
                for(int i = 0; i < volumes.Length; i++) {
                    float progress = (float)(i + 1) / volumes.Length;
                    EditorUtility.DisplayProgressBar("Baking pathfinding grids", volumes[i].gameObject.name, progress);
                    await Bake(volumes[i]);
                }
            }
            finally {
                EditorUtility.ClearProgressBar();
            }
        }

        public static async Task Bake(GridVolume volume) {
            WarnAboutObstaclesOnBakedLayers(volume);

            GridCollection collection = volume.BuildCollection();
            GridNode[][][] bakedGrids = await collection.GenerateGrid();
            byte[] data = GridDataSerializer.SerializeGrid(bakedGrids, collection);

            string folder = string.IsNullOrEmpty(volume.BakedOutputFolder) ? "Assets/PathfindingData" : volume.BakedOutputFolder;
            EnsureFolder(folder);
            string path = $"{folder}/{volume.gameObject.scene.name}_{volume.gameObject.name}.asset";

            PathfindingGridData gridDataAsset = AssetDatabase.LoadAssetAtPath<PathfindingGridData>(path);
            if(gridDataAsset == null) {
                gridDataAsset = ScriptableObject.CreateInstance<PathfindingGridData>();
                AssetDatabase.CreateAsset(gridDataAsset, path);
            }

            gridDataAsset.byteData = data;
            gridDataAsset.gridWidth = collection.Width;
            gridDataAsset.gridHeight = collection.Height;
            gridDataAsset.gridLength = collection.Length;
            EditorUtility.SetDirty(gridDataAsset);
            AssetDatabase.SaveAssets();

            string guid = AssetDatabase.AssetPathToGUID(path);
            EnsureAddressableEntry(guid, path);
            AssignReference(volume, guid);

            Debug.Log($"[GridVolumeBaker] Baked '{path}' ({collection.Width}x{collection.Height}x{collection.Length})", volume);
        }

        private static void EnsureFolder(string folder) {
            if(AssetDatabase.IsValidFolder(folder)) {
                return;
            }

            string[] parts = folder.Split('/');
            string current = parts[0]; // "Assets"
            for(int i = 1; i < parts.Length; i++) {
                string next = $"{current}/{parts[i]}";
                if(!AssetDatabase.IsValidFolder(next)) {
                    AssetDatabase.CreateFolder(current, parts[i]);
                }
                current = next;
            }
        }

        private static void EnsureAddressableEntry(string guid, string path) {
            AddressableAssetSettings settings = AddressableAssetSettingsDefaultObject.Settings;
            if(settings == null) {
                Debug.LogError($"[GridVolumeBaker] Addressables is not initialized in this project (Window > Asset Management > Addressables > Groups). '{path}' was baked but the AssetReference cannot resolve until the asset is made addressable.");
                return;
            }

            if(settings.FindAssetEntry(guid) == null) {
                settings.CreateOrMoveEntry(guid, settings.DefaultGroup);
            }
        }

        private static void AssignReference(GridVolume volume, string guid) {
            SerializedObject serialized = new SerializedObject(volume);
            SerializedProperty guidProperty = serialized.FindProperty("gridData.m_AssetGUID");
            guidProperty.stringValue = guid;
            serialized.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(volume);
        }

        /// <summary>
        /// An object should block through ONE channel: baked layers (never moves) or a
        /// GridObstacle component (dynamic). Both at once means its cells get baked as
        /// static and will not free up when it moves.
        /// </summary>
        private static void WarnAboutObstaclesOnBakedLayers(GridVolume volume) {
            GridObstacle[] obstacles = Object.FindObjectsByType<GridObstacle>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            foreach(GridObstacle obstacle in obstacles) {
                foreach(Collider bakedCollider in obstacle.GetComponentsInChildren<Collider>(true)) {
                    if((volume.BakeLayerMask.value & (1 << bakedCollider.gameObject.layer)) != 0) {
                        Debug.LogWarning($"[GridVolumeBaker] '{obstacle.gameObject.name}' has a GridObstacle AND collider '{bakedCollider.name}' on a layer in the bake mask of '{volume.gameObject.name}'. Its cells will bake as static and won't unblock when it moves. Use one blocking channel only.", obstacle);
                        break;
                    }
                }
            }
        }
    }
}
