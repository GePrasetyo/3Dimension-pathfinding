using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace Majinfwork.Pathfinding.Tests {
    public class PathFollowerTests {
        private const float dt = 1f / 60f;
        private const float accel = 8f;
        private const float eps = 0.02f;

        private GameObject body;
        private PathFollower follower;

        [SetUp]
        public void SetUp() {
            body = new GameObject("FollowerBody");
            follower = new PathFollower {
                maxSpeed = 3f,
                acceleration = accel,
                lookaheadDistance = 1.5f
            };
            follower.Initialize(body.transform);
            follower.UpdateSpeed(1f);
        }

        [TearDown]
        public void TearDown() {
            Object.DestroyImmediate(body);
        }

        [Test]
        public void ArrivesAtEndOfLShapedPath() {
            follower.InputPathToMove(new List<Vector3> { new Vector3(4, 0, 0), new Vector3(4, 0, 4) });

            bool arrived = Simulate(2000, new Vector3(4, 0, 4), out _, out _);
            Assert.IsTrue(arrived);
        }

        [Test]
        public void VelocityChangeIsAccelerationLimitedEveryFrame() {
            follower.InputPathToMove(new List<Vector3> { new Vector3(4, 0, 0), new Vector3(4, 0, 4) });

            Simulate(2000, new Vector3(4, 0, 4), out float maxVelocityStep, out float maxSpeedSeen);
            Assert.LessOrEqual(maxVelocityStep, accel * dt + eps, "per-frame velocity change exceeded the acceleration limit");
            Assert.LessOrEqual(maxSpeedSeen, 3f + eps, "speed exceeded maxSpeed");
        }

        [Test]
        public void RepathKeepsVelocityContinuous() {
            follower.InputPathToMove(new List<Vector3> { new Vector3(20, 0, 0) });
            for(int i = 0; i < 60; i++) {
                follower.Move(dt, out _);
            }

            Vector3 before = body.transform.position;
            follower.Move(dt, out _);
            Vector3 cruiseVelocity = (body.transform.position - before) / dt;
            Assert.Greater(cruiseVelocity.magnitude, 2.5f, "should be cruising before the repath");

            // Retarget mid-flight, like a drone chasing a strafing player
            follower.InputPathToMove(new List<Vector3> { body.transform.position + new Vector3(2, 0, 6) });

            Vector3 beforeStep = body.transform.position;
            follower.Move(dt, out _);
            Vector3 afterVelocity = (body.transform.position - beforeStep) / dt;

            Assert.LessOrEqual((afterVelocity - cruiseVelocity).magnitude, accel * dt + eps,
                "repath must not snap the velocity - only one frame of acceleration may apply");
        }

        [Test]
        public void EmptyPathStops() {
            follower.InputPathToMove(new List<Vector3> { new Vector3(20, 0, 0) });
            for(int i = 0; i < 60; i++) {
                follower.Move(dt, out _);
            }
            Assert.Greater(follower.speed, 0f);

            follower.InputPathToMove(new List<Vector3>());
            Assert.AreEqual(0f, follower.speed);

            Vector3 before = body.transform.position;
            follower.Move(dt, out Vector3? direction);
            Assert.IsNull(direction);
            Assert.AreEqual(before, body.transform.position);
        }

        private bool Simulate(int maxSteps, Vector3 target, out float maxVelocityStep, out float maxSpeedSeen) {
            Vector3 previousVelocity = Vector3.zero;
            maxVelocityStep = 0f;
            maxSpeedSeen = 0f;

            for(int i = 0; i < maxSteps; i++) {
                Vector3 before = body.transform.position;
                follower.Move(dt, out _);
                Vector3 velocity = (body.transform.position - before) / dt;

                maxVelocityStep = Mathf.Max(maxVelocityStep, (velocity - previousVelocity).magnitude);
                maxSpeedSeen = Mathf.Max(maxSpeedSeen, velocity.magnitude);
                previousVelocity = velocity;

                if((body.transform.position - target).magnitude < 0.15f) {
                    return true;
                }
            }

            return false;
        }
    }
}
