using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

namespace Majinfwork.Pathfinding.Tests {
    /// <summary>
    /// 7x7x7 grid with a solid wall plane at x == 3 and a single opening at (3,6,6).
    /// Small enough that every time-sliced UniTask completes synchronously.
    /// </summary>
    public class AStarSearchTests {
        private const int size = 7;
        private const float pd = 1f;
        private static readonly Vector3 origin = new Vector3(-size, -size, -size) / 2f * pd;

        private GridCollection collection;
        private GridNode[][][] grids;
        private GridAStarAgent agent;
        private GameObject agentObject;
        private PathAgentType agentType;

        private static Vector3 Cell(int x, int y, int z) => origin + new Vector3(x, y, z) * pd;

        [SetUp]
        public void SetUp() {
            agentType = ScriptableObject.CreateInstance<PathAgentType>();
            collection = new GridCollection(size, size, size, pd, Vector3.zero, agentType, 0);

            grids = new GridNode[size][][];
            for(int x = 0; x < size; x++) {
                grids[x] = new GridNode[size][];
                for(int y = 0; y < size; y++) {
                    grids[x][y] = new GridNode[size];
                    for(int z = 0; z < size; z++) {
                        GridNode node = new GridNode();
                        node.coords = new Vector3Int(x, y, z);
                        node.worldPosition = Cell(x, y, z);
                        node.BlockedArea(x == 3 && !(y == 6 && z == 6));
                        grids[x][y][z] = node;
                    }
                }
            }

            collection.InitializeGrid(grids);
            collection.ComputeIslands().GetAwaiter().GetResult();

            agentObject = new GameObject("TestAgent");
            agent = new GridAStarAgent(agentObject.transform, agentType);
            typeof(GridAStarAgent)
                .GetField("gridCollection", BindingFlags.NonPublic | BindingFlags.Instance)
                .SetValue(agent, collection);
        }

        [TearDown]
        public void TearDown() {
            collection.Dispose();
            Object.DestroyImmediate(agentObject);
            Object.DestroyImmediate(agentType);
        }

        [Test]
        public void FindsPathThroughOpening() {
            List<Vector3> path = new List<Vector3>();
            bool found = agent.GetPath(Cell(1, 3, 3), Cell(5, 3, 3), ref path);

            Assert.IsTrue(found);
            Assert.Greater(path.Count, 1);
        }

        [Test]
        public void EveryPrunedSegmentHasLineOfSight() {
            List<Vector3> path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Cell(1, 3, 3), Cell(5, 3, 3), ref path));

            for(int i = 1; i < path.Count; i++) {
                Assert.IsTrue(collection.HasLineOfSight(path[i - 1], path[i]),
                    $"Segment {i - 1}->{i} crosses blocked cells");
            }
        }

        [Test]
        public void LineOfSightBlockedThroughWall() {
            Assert.IsFalse(collection.HasLineOfSight(Cell(2, 3, 3), Cell(4, 3, 3)));
        }

        [Test]
        public void LineOfSightClearInOpenSpace() {
            Assert.IsTrue(collection.HasLineOfSight(Cell(0, 0, 0), Cell(2, 6, 6)));
        }

        [Test]
        public void StaticIslandsRejectSealedGoalInstantly() {
            grids[3][6][6].BlockedArea(true); // close the opening statically
            collection.ComputeIslands().GetAwaiter().GetResult();

            collection.GetClosestPointWorldSpace(Cell(1, 3, 3), out GridNode a, getClosestValid: false);
            collection.GetClosestPointWorldSpace(Cell(5, 3, 3), out GridNode b, getClosestValid: false);

            Assert.IsFalse(collection.AreConnected(a, b));

            List<Vector3> path = new List<Vector3>();
            Assert.IsFalse(agent.GetPath(Cell(1, 3, 3), Cell(5, 3, 3), ref path));
        }

        [Test]
        public void DynamicSeal_ProbeProvesIt_WhileIslandMapIsStale() {
            grids[3][6][6].AddDynamicBlock();
            GridLocator.BumpObstacleVersion();

            collection.GetClosestPointWorldSpace(Cell(1, 3, 3), out GridNode a, getClosestValid: false);
            collection.GetClosestPointWorldSpace(Cell(5, 3, 3), out GridNode b, getClosestValid: false);

            Assert.IsFalse(collection.HasFreshDynamicIslands(), "map should be stale after version bump");
            Assert.IsTrue(collection.GoalPocketSealed(b, a), "probe should prove the seal");
            Assert.IsFalse(collection.GoalPocketSealed(b, a, 4), "tiny budget must be inconclusive, not a guess");

            List<Vector3> path = new List<Vector3>();
            Assert.IsFalse(agent.GetPath(Cell(1, 3, 3), Cell(5, 3, 3), ref path));

            grids[3][6][6].RemoveDynamicBlock();
            GridLocator.BumpObstacleVersion();
        }

        [Test]
        public void DynamicSeal_FreshIslandMapRejectsInConstantTime() {
            grids[3][6][6].AddDynamicBlock();
            GridLocator.BumpObstacleVersion();
            collection.ComputeDynamicIslands().GetAwaiter().GetResult();

            collection.GetClosestPointWorldSpace(Cell(1, 3, 3), out GridNode a, getClosestValid: false);
            collection.GetClosestPointWorldSpace(Cell(5, 3, 3), out GridNode b, getClosestValid: false);

            Assert.IsTrue(collection.HasFreshDynamicIslands());
            Assert.IsFalse(collection.AreConnected(a, b));

            grids[3][6][6].RemoveDynamicBlock();
            GridLocator.BumpObstacleVersion();

            List<Vector3> path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Cell(1, 3, 3), Cell(5, 3, 3), ref path), "reopened gap should path again");
        }

        [Test]
        public void DynamicBlockRefcountSurvivesOverlapAndStaticStaysBlocked() {
            GridNode cell = grids[1][1][1];
            cell.AddDynamicBlock();
            cell.AddDynamicBlock();
            cell.RemoveDynamicBlock();
            Assert.IsTrue(cell.invalid, "one of two blocks released - still blocked");
            cell.RemoveDynamicBlock();
            Assert.IsFalse(cell.invalid);

            GridNode wall = grids[3][0][0];
            wall.AddDynamicBlock();
            wall.RemoveDynamicBlock();
            Assert.IsTrue(wall.invalid, "dynamic release must never clear static blocking");
        }
    }
}
