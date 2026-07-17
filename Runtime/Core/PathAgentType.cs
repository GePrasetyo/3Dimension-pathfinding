using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Identity asset for an agent category. Create one asset per kind of flying
    /// agent (e.g. "Drone", "Wasp", "Boss") and reference the same asset from the
    /// GridVolume and the agents that should use its grid. Runtime lookups key on
    /// the asset reference itself, so comparisons are as fast as integer keys.
    /// </summary>
    [CreateAssetMenu(menuName = "Majinfwork/Pathfinding/Agent Type", fileName = "PathAgentType")]
    public class PathAgentType : ScriptableObject {
        [TextArea] public string description;
    }
}
