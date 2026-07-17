using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Baked grid payload. Created by the GridVolume bake button; the binary format
    /// is a header (magic, version, dims, bake origin, spacing) plus one blocked bit
    /// per cell - everything else is derivable at load time.
    /// </summary>
    [CreateAssetMenu(menuName = "Majinfwork/Pathfinding/Grid Data", fileName = "PathfindingGridData")]
    [PreferBinarySerialization]
    public class PathfindingGridData : ScriptableObject {
        public int gridWidth = 0;
        public int gridHeight = 0;
        public int gridLength = 0;
        public byte[] byteData;
    }
}
