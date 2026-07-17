using UnityEditor;
using UnityEngine;

namespace Majinfwork.Pathfinding.Editor {
    [CustomEditor(typeof(GridVolume))]
    public class GridVolumeEditor : UnityEditor.Editor {
        public override void OnInspectorGUI() {
            DrawDefaultInspector();

            if(GUILayout.Button("Bake grid data for this volume")) {
                _ = GridVolumeBaker.Bake((GridVolume)target);
            }

            if(GUILayout.Button("Bake grid data for ALL volumes in the scene")) {
                _ = GridVolumeBaker.BakeAllInScene();
            }
        }
    }
}
