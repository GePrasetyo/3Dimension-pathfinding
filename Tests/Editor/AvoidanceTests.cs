using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace Majinfwork.Pathfinding.Tests {
    public class AvoidanceTests {
        private const float dt = 1f / 60f;
        private const float radius = 1.5f;
        private const float lookahead = 2f;
        private const float strength = 2f;

        private GameObject bodyA, bodyB;
        private PathFollower followerA, followerB;

        [SetUp]
        public void SetUp() {
            bodyA = new GameObject("AgentA");
            bodyB = new GameObject("AgentB");
            followerA = MakeFollower(bodyA.transform);
            followerB = MakeFollower(bodyB.transform);
        }

        [TearDown]
        public void TearDown() {
            Object.DestroyImmediate(bodyA);
            Object.DestroyImmediate(bodyB);
        }

        private static PathFollower MakeFollower(Transform body) {
            PathFollower follower = new PathFollower {
                maxSpeed = 2.5f,
                acceleration = 8f,
                lookaheadDistance = 1.5f
            };
            follower.Initialize(body);
            follower.UpdateSpeed(1f);
            return follower;
        }

        [Test]
        public void HeadOnWithoutAvoidanceCollides() {
            float minDistance = RunScenario(
                new Vector3(0, 0, 0), new Vector3(10, 0, 0),
                new Vector3(10, 0, 0), new Vector3(0, 0, 0),
                useAvoidance: false, out bool bothArrived);

            Assert.Less(minDistance, 0.3f, "scenario must be a genuine collision course");
            Assert.IsTrue(bothArrived);
        }

        [Test]
        public void HeadOnWithAvoidanceKeepsClearance() {
            float minDistance = RunScenario(
                new Vector3(0, 0, 0), new Vector3(10, 0, 0),
                new Vector3(10, 0, 0), new Vector3(0, 0, 0),
                useAvoidance: true, out bool bothArrived);

            Assert.Greater(minDistance, 0.6f);
            Assert.IsTrue(bothArrived, "avoidance must not prevent arrival");
        }

        [Test]
        public void PerpendicularCrossingWithAvoidanceKeepsClearance() {
            float minDistance = RunScenario(
                new Vector3(0, 0, 5), new Vector3(10, 0, 5),
                new Vector3(5, 0, 0), new Vector3(5, 0, 10),
                useAvoidance: true, out bool bothArrived);

            Assert.Greater(minDistance, 0.6f);
            Assert.IsTrue(bothArrived);
        }

        private float RunScenario(Vector3 startA, Vector3 targetA, Vector3 startB, Vector3 targetB, bool useAvoidance, out bool bothArrived) {
            bodyA.transform.position = startA;
            bodyB.transform.position = startB;
            followerA.CancelMove();
            followerB.CancelMove();
            followerA.InputPathToMove(new List<Vector3> { targetA });
            followerB.InputPathToMove(new List<Vector3> { targetB });

            float minDistance = float.MaxValue;

            for(int i = 0; i < 1500; i++) {
                if(useAvoidance) {
                    Vector3 biasA = Vector3.ClampMagnitude(PathFollower.ComputeAvoidance(
                        bodyA.transform.position, followerA.currentVelocity,
                        bodyB.transform.position, followerB.currentVelocity, radius, lookahead) * strength, strength);
                    Vector3 biasB = Vector3.ClampMagnitude(PathFollower.ComputeAvoidance(
                        bodyB.transform.position, followerB.currentVelocity,
                        bodyA.transform.position, followerA.currentVelocity, radius, lookahead) * strength, strength);
                    followerA.SetAvoidance(biasA);
                    followerB.SetAvoidance(biasB);
                }

                followerA.Move(dt, out _);
                followerB.Move(dt, out _);

                float distance = (bodyA.transform.position - bodyB.transform.position).magnitude;
                if(distance < minDistance) {
                    minDistance = distance;
                }
            }

            bothArrived = (bodyA.transform.position - targetA).magnitude < 0.3f
                && (bodyB.transform.position - targetB).magnitude < 0.3f;
            return minDistance;
        }
    }
}
