using UnityEditor;
using UnityEngine;

namespace Majinfwork.Pathfinding.Editor {
    /// <summary>A <see cref="PathAgentType"/> as a popup of its names, like a layer field.</summary>
    [CustomPropertyDrawer(typeof(PathAgentType))]
    public class PathAgentTypeDrawer : PropertyDrawer {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
            SerializedProperty value = property.FindPropertyRelative("value");
            int current = value.intValue;
            int count = Mathf.Max(PathAgentTypeSettings.instance.Count, current + 1);
            int edit = count + 1;
            var options = new GUIContent[count + 2];
            for(int i = 0; i < count; i++) {
                options[i] = new GUIContent($"{i}: {PathAgentTypeSettings.NameOf(i)}");
            }

            options[count] = new GUIContent(string.Empty);
            options[edit] = new GUIContent("Edit Agent Types...");

            label = EditorGUI.BeginProperty(position, label, property);
            EditorGUI.BeginChangeCheck();
            int picked = EditorGUI.Popup(position, label, current, options);
            if(EditorGUI.EndChangeCheck()) {
                if(picked == edit) {
                    SettingsService.OpenProjectSettings(PathAgentTypeSettings.SettingsPath);
                }
                else if(picked < count) {
                    value.intValue = picked;
                }
            }

            EditorGUI.EndProperty();
        }
    }
}
