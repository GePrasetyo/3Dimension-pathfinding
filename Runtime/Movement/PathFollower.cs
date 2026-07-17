using System;
using System.Collections.Generic;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Pure-pursuit path follower: steers a persistent velocity toward a lookahead
    /// point on the waypoint polyline under an acceleration limit. Velocity survives
    /// repaths, so replacing the path never snaps the direction of travel; turn
    /// smoothness comes from the acceleration limit instead of curve geometry.
    /// Embed as a [SerializeField] inside the host's motor component.
    /// </summary>
    [Serializable]
    public class PathFollower {
        [field: SerializeField, Min(0.1f)] public float maxSpeed { get; set; } = 2.5f;
        [field: SerializeField, Min(0.1f)] public float acceleration { get; set; } = 8f;
        [field: SerializeField, Min(0.1f)] public float lookaheadDistance { get; set; } = 1.5f;

        public float speed { private set; get; }
        public Vector3 currentVelocity => velocity;

        /// <summary>
        /// Global speed multiplier (difficulty, buffs, slows - host's business).
        /// </summary>
        public float speedFactor { get; set; } = 1f;

        private const float arrivalTolerance = 0.05f;

        private List<Vector3> pathPoints = new List<Vector3>();
        private List<float> cumulativeLengths = new List<float>();
        private Transform bodyTransform;
        private Vector3 velocity;
        private Vector3 avoidanceVelocity;
        private int segmentIndex;
        private float accelerationMultiplier = 1f;
        private bool initialized;

        public void Initialize(Transform transform) {
            initialized = true;
            bodyTransform = transform;
        }

        public void Disable() {
            initialized = false;
            CancelMove();
        }

        public void InputPathToMove(List<Vector3> path) {
            if(initialized == false) {
                return;
            }

            // An empty path means stop.
            if(path == null || path.Count == 0) {
                CancelMove();
                return;
            }

            pathPoints.Clear();
            pathPoints.Add(bodyTransform.position);
            for(int i = 0; i < path.Count; i++) {
                pathPoints.Add(path[i]);
            }

            cumulativeLengths.Clear();
            cumulativeLengths.Add(0f);
            for(int i = 1; i < pathPoints.Count; i++) {
                cumulativeLengths.Add(cumulativeLengths[i - 1] + Vector3.Distance(pathPoints[i - 1], pathPoints[i]));
            }

            segmentIndex = 0;
            // velocity is deliberately kept: continuity across repaths is the point
        }

        public void CancelMove() {
            pathPoints.Clear();
            cumulativeLengths.Clear();
            segmentIndex = 0;
            velocity = Vector3.zero;
            avoidanceVelocity = Vector3.zero;
            speed = 0f;
        }

        public void UpdateSpeed(float multiplierSpeed) {
            accelerationMultiplier = Mathf.Max(0.1f, multiplierSpeed);
        }

        /// <summary>
        /// Desired-velocity bias from local avoidance, applied inside Move under the
        /// same acceleration limit as path following so dodges stay smooth and can
        /// never exceed physical movement limits.
        /// </summary>
        public void SetAvoidance(Vector3 avoidance) {
            avoidanceVelocity = avoidance;
        }

        /// <summary>
        /// Predictive pairwise avoidance (Reynolds' unaligned collision avoidance):
        /// finds the closest approach of the two agents within the lookahead given
        /// current velocities and, if it violates the radius, returns a steer-away
        /// direction scaled by urgency (0..1). Time zero handles the already-close
        /// case, so this also covers plain separation. The host sums this over its
        /// live agents and feeds the total into SetAvoidance.
        /// </summary>
        public static Vector3 ComputeAvoidance(Vector3 position, Vector3 velocity, Vector3 otherPosition, Vector3 otherVelocity, float radius, float lookahead) {
            Vector3 relativePosition = otherPosition - position;
            Vector3 relativeVelocity = otherVelocity - velocity;
            float sqrRelativeSpeed = relativeVelocity.sqrMagnitude;

            float timeToClosest = sqrRelativeSpeed > 0.0001f
                ? Mathf.Clamp(-Vector3.Dot(relativePosition, relativeVelocity) / sqrRelativeSpeed, 0f, lookahead)
                : 0f;

            Vector3 closestOffset = relativePosition + relativeVelocity * timeToClosest;
            float closestDistance = closestOffset.magnitude;

            if(closestDistance >= radius) {
                return Vector3.zero;
            }

            // Sooner threats matter more; even at full lookahead keep half strength.
            float urgency = (1f - closestDistance / radius) * (1f - 0.5f * timeToClosest / Mathf.Max(lookahead, 0.0001f));

            Vector3 away;
            if(closestDistance < 0.05f) {
                // Dead-center collision course: deterministic side-step. Head-on agents
                // have opposite forward vectors, so each dodges to its own right.
                Vector3 forward = velocity.sqrMagnitude > 0.01f ? velocity.normalized : Vector3.forward;
                away = (Vector3.Cross(forward, Vector3.up) + Vector3.up * 0.5f).normalized;
            }
            else {
                away = -closestOffset / closestDistance;
            }

            return away * urgency;
        }

        public void Move(float deltaTime, out Vector3? direction) {
            direction = null;

            if(initialized == false || pathPoints.Count < 2) {
                return;
            }

            Vector3 position = bodyTransform.position;
            AdvanceSegment(position);

            float remaining = RemainingDistance(position);
            if(remaining <= arrivalTolerance) {
                CancelMove();
                return;
            }

            float effectiveAcceleration = acceleration * accelerationMultiplier;
            Vector3 carrot = PointAlongPath(TraveledDistance(position) + lookaheadDistance);

            // Braking-limited target speed: v = sqrt(2 * a * d) reaches zero exactly at the end.
            float targetSpeed = Mathf.Min(maxSpeed * speedFactor, Mathf.Sqrt(2f * effectiveAcceleration * remaining));
            Vector3 toCarrot = carrot - position;
            Vector3 desiredVelocity = toCarrot.sqrMagnitude > 0.0001f ? toCarrot.normalized * targetSpeed : Vector3.zero;
            desiredVelocity = Vector3.ClampMagnitude(desiredVelocity + avoidanceVelocity, maxSpeed * speedFactor);

            velocity = Vector3.MoveTowards(velocity, desiredVelocity, effectiveAcceleration * deltaTime);
            bodyTransform.position = position + velocity * deltaTime;
            speed = velocity.magnitude;

            if(speed > 0.001f) {
                direction = velocity;
            }
        }

        private void AdvanceSegment(Vector3 position) {
            while(segmentIndex < pathPoints.Count - 2 && ProjectOnSegment(position, segmentIndex) >= 1f) {
                segmentIndex++;
            }
        }

        private float ProjectOnSegment(Vector3 position, int index) {
            Vector3 a = pathPoints[index];
            Vector3 ab = pathPoints[index + 1] - a;
            float sqrLength = ab.sqrMagnitude;

            if(sqrLength < 0.000001f) {
                return 1f;
            }

            return Vector3.Dot(position - a, ab) / sqrLength;
        }

        private float TraveledDistance(Vector3 position) {
            float t = Mathf.Clamp01(ProjectOnSegment(position, segmentIndex));
            float segmentLength = cumulativeLengths[segmentIndex + 1] - cumulativeLengths[segmentIndex];
            return cumulativeLengths[segmentIndex] + t * segmentLength;
        }

        private float RemainingDistance(Vector3 position) {
            if(segmentIndex >= pathPoints.Count - 2) {
                // Last segment: use the true distance so braking accounts for lateral offset.
                return Vector3.Distance(position, pathPoints[pathPoints.Count - 1]);
            }

            return cumulativeLengths[cumulativeLengths.Count - 1] - TraveledDistance(position);
        }

        private Vector3 PointAlongPath(float distance) {
            if(distance >= cumulativeLengths[cumulativeLengths.Count - 1]) {
                return pathPoints[pathPoints.Count - 1];
            }

            int index = segmentIndex;
            while(index < pathPoints.Count - 2 && cumulativeLengths[index + 1] < distance) {
                index++;
            }

            float segmentStart = cumulativeLengths[index];
            float segmentLength = cumulativeLengths[index + 1] - segmentStart;
            float t = segmentLength > 0.0001f ? Mathf.Clamp01((distance - segmentStart) / segmentLength) : 1f;
            return Vector3.Lerp(pathPoints[index], pathPoints[index + 1], t);
        }
    }
}
