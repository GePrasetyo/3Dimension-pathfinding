using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace Majinfwork.Pathfinding.Tests {
    /// <summary>
    /// A 12x6x12 walking grid, 1 m cells, for a walker 2 m tall that climbs 1 m and drops 2 m. The ground is the y == 0
    /// layer, so walkers stand at y == 1.
    /// - A wall at x == 5 with one doorway at z == 2.
    /// - A 1 m step (x 2-3, z 8-9): climbable.
    /// - A 2 m block (x 8-9, z 8-9): too high to climb, low enough to drop from.
    /// - A 1 m low ceiling over (1, z 4): nobody stands under it.
    /// - A hole in the floor at (7, z 4).
    /// - A 4 m platform (x 10-11, z 0-1): neither climbable nor droppable, its own island.
    /// </summary>
    public class WalkingSearchTests {
        private const int Width = 12;
        private const int Height = 6;
        private const int Length = 12;
        private const float CellSize = 1f;
        private static readonly Vector3 Origin = new Vector3(-Width, -Height, -Length) / 2f * CellSize;
        private static readonly WalkSettings Walker = new WalkSettings { bodyHeight = 2f, stepHeight = 1f, dropHeight = 2f };

        private GridCollection collection;
        private GridAStarAgent agent;
        private GameObject agentObject;

        private static Vector3 Cell(int x, int y, int z) => Origin + new Vector3(x, y, z) * CellSize;

        /// <summary>A point on the floor of the cell a walker stands in at (x, y, z).</summary>
        private static Vector3 Floor(int x, int y, int z) => Cell(x, y, z) - Vector3.up * (CellSize * 0.5f);

        private static bool Blocked(int x, int y, int z) {
            if (y == 0) {
                return !(x == 7 && z == 4);
            }

            bool wall = x == 5 && y <= 5 && !(z == 2 && y <= 2);
            bool step = x >= 2 && x <= 3 && z >= 8 && z <= 9 && y == 1;
            bool block = x >= 8 && x <= 9 && z >= 8 && z <= 9 && y <= 2;
            bool ceiling = x == 1 && z == 4 && y == 2;
            bool platform = x >= 10 && z <= 1 && y <= 4;
            return wall || step || block || ceiling || platform;
        }

        [SetUp]
        public void SetUp() {
            collection = new GridCollection(Width, Height, Length, CellSize, Vector3.zero, new PathAgentType(7), 0, PathMovement.Walk, Walker);
            GridNode[][][] grids = new GridNode[Width][][];
            for (int x = 0; x < Width; x++) {
                grids[x] = new GridNode[Height][];
                for (int y = 0; y < Height; y++) {
                    grids[x][y] = new GridNode[Length];
                    for (int z = 0; z < Length; z++) {
                        GridNode node = new GridNode();
                        node.coords = new Vector3Int(x, y, z);
                        node.worldPosition = Cell(x, y, z);
                        node.BlockedArea(Blocked(x, y, z));
                        grids[x][y][z] = node;
                    }
                }
            }

            collection.InitializeGrid(grids);
            collection.ComputeIslands().GetAwaiter().GetResult();
            agentObject = new GameObject("Walker");
            agent = new GridAStarAgent(agentObject.transform, collection.AgentType);
            agent.SetGrid(collection);
        }

        [TearDown]
        public void TearDown() {
            collection.Dispose();
            Object.DestroyImmediate(agentObject);
        }

        private Vector3Int CellOf(Vector3 point) => Vector3Int.RoundToInt((point - Origin) / CellSize);

        [Test]
        public void StandsOnFloorsOnly() {
            Assert.IsTrue(collection.IsStandable(1, 1, 1), "on the ground");
            Assert.IsFalse(collection.IsStandable(1, 2, 1), "in the air");
            Assert.IsFalse(collection.IsStandable(7, 1, 4), "over the hole");
            Assert.IsFalse(collection.IsStandable(1, 1, 4), "under the low ceiling");
            Assert.IsTrue(collection.IsStandable(2, 2, 8), "on the step");
            Assert.IsTrue(collection.IsStandable(8, 3, 8), "on the block");
        }

        [Test]
        public void WalksThroughTheDoorwayAndStandsAllTheWay() {
            List<Vector3> path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Floor(1, 1, 6), Floor(8, 1, 6), ref path));

            bool throughDoor = false;
            foreach (Vector3 point in path) {
                Vector3Int cell = CellOf(point);
                Assert.IsTrue(collection.IsStandable(cell.x, cell.y, cell.z), $"waypoint {cell} is not stood on");
                throughDoor |= cell.x == 5 && cell.z == 2;
            }

            for (int i = 1; i < path.Count; i++) {
                Assert.IsTrue(collection.HasWalkableLineOfSight(path[i - 1], path[i]), $"segment {i - 1}->{i} leaves the floor");
            }

            Assert.IsTrue(throughDoor || path.Exists(p => CellOf(p).x > 5), "path must reach past the wall");
            Assert.AreEqual(new Vector3Int(8, 1, 6), CellOf(path[path.Count - 1]));
        }

        [Test]
        public void ClimbsAOneMetreStep() {
            List<Vector3> path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Floor(1, 1, 9), Floor(3, 2, 9), ref path));
            Assert.AreEqual(new Vector3Int(3, 2, 9), CellOf(path[path.Count - 1]));
        }

        [Test]
        public void DropsFromTheBlockButCannotClimbIt() {
            List<Vector3> path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Floor(8, 3, 8), Floor(7, 1, 8), ref path), "a 2 m drop is allowed");
            Assert.IsFalse(agent.GetPath(Floor(7, 1, 8), Floor(8, 3, 8), ref path), "a 2 m climb is not");
        }

        [Test]
        public void WalksAroundTheLowCeilingAndTheHole() {
            List<Vector3> path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Floor(1, 1, 3), Floor(1, 1, 5), ref path));
            Assert.IsFalse(path.Exists(p => CellOf(p) == new Vector3Int(1, 1, 4)), "nobody fits under the ceiling");

            Assert.IsTrue(agent.GetPath(Floor(7, 1, 3), Floor(7, 1, 5), ref path));
            Assert.IsFalse(path.Exists(p => CellOf(p).x == 7 && CellOf(p).z == 4), "nobody stands over the hole");
        }

        [Test]
        public void ThePlatformIsItsOwnIsland() {
            Assert.IsTrue(collection.TryGetStandable(Floor(1, 1, 1), 1, out GridNode ground));
            Assert.IsTrue(collection.TryGetStandable(Floor(10, 5, 0), 1, out GridNode platform));
            Assert.AreEqual(new Vector3Int(10, 5, 0), platform.coords);
            Assert.IsFalse(collection.AreConnected(ground, platform));

            List<Vector3> path = new List<Vector3>();
            Assert.IsFalse(agent.GetPath(Floor(1, 1, 1), Floor(10, 5, 0), ref path));
        }

        [Test]
        public void AFootOnTheFloorFindsTheCellItStandsIn() {
            Assert.IsTrue(collection.TryGetStandable(Floor(3, 1, 3) + new Vector3(0.2f, 0f, -0.3f), 1, out GridNode node));
            Assert.AreEqual(new Vector3Int(3, 1, 3), node.coords);
        }

        [Test]
        public void FlyingGridsKeepFlying() {
            GridCollection flying = new GridCollection(Width, Height, Length, CellSize, Vector3.zero, new PathAgentType(8), 0);
            Assert.AreEqual(PathMovement.Fly, flying.Movement);
            Assert.IsFalse(flying.IsWalking);
            flying.Dispose();
        }
    }
}
