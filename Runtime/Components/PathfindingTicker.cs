using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Zero-setup driver: ticks obstacles and island maintenance once per frame.
    /// If the host wants to own the cadence (or tick from a custom loop), set
    /// autoTick to false before the first scene loads and call
    /// GridObstacle.TickAll() once per frame instead.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public sealed class PathfindingTicker : MonoBehaviour {
        public static bool autoTick = true;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Bootstrap() {
            if(!autoTick) {
                return;
            }

            GameObject go = new GameObject("[Pathfinding3D Ticker]");
            DontDestroyOnLoad(go);
            go.AddComponent<PathfindingTicker>();
        }

        private void Update() {
            GridObstacle.TickAll();
        }
    }
}
