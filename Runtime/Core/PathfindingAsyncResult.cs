using System.Collections.Generic;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    public record PathfindingAsyncResult(bool success, Vector3 targetPoint, List<Vector3> pathPoints, float distance);
}
