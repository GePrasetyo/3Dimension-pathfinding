using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Majinfwork.Pathfinding.Editor {
    /// <summary>
    /// The agent types' names, editor-only like the tag manager: a name's index is
    /// its type's byte. Saved in ProjectSettings, so it never ships.
    /// </summary>
    [FilePath("ProjectSettings/PathAgentTypes.asset", FilePathAttribute.Location.ProjectFolder)]
    public class PathAgentTypeSettings : ScriptableSingleton<PathAgentTypeSettings> {
        public const string SettingsPath = "Project/Pathfinding Agent Types";
        private const int MaxTypes = byte.MaxValue + 1;

        [SerializeField] private List<string> names = new List<string>();

        public int Count => names.Count;

        public static string NameOf(int value) {
            List<string> names = instance.names;
            return value < names.Count && !string.IsNullOrEmpty(names[value]) ? names[value] : $"Type {value}";
        }

        [SettingsProvider]
        private static SettingsProvider CreateProvider() {
            return new SettingsProvider(SettingsPath, SettingsScope.Project) {
                guiHandler = _ => instance.OnGUI(),
                keywords = new[] { "pathfinding", "agent", "grid" }
            };
        }

        private void OnGUI() {
            EditorGUI.BeginChangeCheck();
            for(int i = 0; i < names.Count; i++) {
                names[i] = EditorGUILayout.TextField($"Type {i}", names[i]);
            }

            bool changed = EditorGUI.EndChangeCheck();
            using(new EditorGUILayout.HorizontalScope()) {
                GUILayout.FlexibleSpace();
                using(new EditorGUI.DisabledScope(names.Count >= MaxTypes)) {
                    if(GUILayout.Button("+", GUILayout.Width(24))) {
                        names.Add(string.Empty);
                        changed = true;
                    }
                }

                using(new EditorGUI.DisabledScope(names.Count == 0)) {
                    if(GUILayout.Button("-", GUILayout.Width(24))) {
                        names.RemoveAt(names.Count - 1);
                        changed = true;
                    }
                }
            }

            if(changed) {
                Save(true);
            }
        }
    }
}
