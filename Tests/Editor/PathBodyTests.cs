using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace Majinfwork.Pathfinding.Tests {
    /// <summary>
    /// Sized flyers (<see cref="PathBody"/>) on a 7x7x7 grid, cells 1 m apart, with a solid wall plane at x == 3 and an
    /// opening each test makes in it.
    /// </summary>
    public class PathBodyTests {
        private const int size = 7;
        private const float pd = 1f;
        private static readonly Vector3 origin = new Vector3(-size, -size, -size) / 2f * pd;
        /// <summary>A Sentry: 0.6 m across, 1.8 m tall (two cells on this grid: its lowest and the one above).</summary>
        private static readonly PathBody tall = new PathBody(0.6f, 1.8f);

        private GridCollection collection;
        private GridNode[][][] grids;
        private GridAStarAgent agent;
        private GameObject agentObject;

        private static Vector3 Cell(int x, int y, int z) => origin + new Vector3(x, y, z) * pd;

        private void Build(params Vector3Int[] opening) {
            var open = new HashSet<Vector3Int>(opening);
            var agentType = new PathAgentType(0);
            collection = new GridCollection(size, size, size, pd, Vector3.zero, agentType, 0);
            grids = new GridNode[size][][];
            for(int x = 0; x < size; x++) {
                grids[x] = new GridNode[size][];
                for(int y = 0; y < size; y++) {
                    grids[x][y] = new GridNode[size];
                    for(int z = 0; z < size; z++) {
                        var node = new GridNode();
                        node.coords = new Vector3Int(x, y, z);
                        node.worldPosition = Cell(x, y, z);
                        node.BlockedArea(x == 3 && !open.Contains(node.coords));
                        grids[x][y][z] = node;
                    }
                }
            }

            collection.InitializeGrid(grids);
            collection.ComputeIslands().GetAwaiter().GetResult();
            agentObject = new GameObject("TestAgent");
            agent = new GridAStarAgent(agentObject.transform, agentType);
            agent.SetGrid(collection);
        }

        [TearDown]
        public void TearDown() {
            collection?.Dispose();
            if(agentObject != null) {
                Object.DestroyImmediate(agentObject);
            }
        }

        [Test]
        public void CellsCoverTheBodyWithTheBakesFreeBox() {
            // A free cell holds 1.2 m (one cell plus the bake's margin both ways): 1.8 m needs one more cell above.
            tall.Cells(1f, out int up, out int ring);
            Assert.AreEqual(1, up);
            Assert.AreEqual(0, ring);
            // Half-metre cells hold 0.7 m: 1.8 m needs three more above.
            tall.Cells(0.5f, out up, out ring);
            Assert.AreEqual(3, up);
            Assert.AreEqual(0, ring);
            // 1.5 m across sticks out of a 1.2 m box: a ring of one cell round it.
            new PathBody(1.5f, 1f).Cells(1f, out up, out ring);
            Assert.AreEqual(0, up);
            Assert.AreEqual(1, ring);
            PathBody.Point.Cells(1f, out up, out ring);
            Assert.AreEqual(0, up);
            Assert.AreEqual(0, ring);
        }

        [Test]
        public void OneCellOpeningLetsAPointThroughButNotATallBody() {
            Build(new Vector3Int(3, 3, 3));
            var path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Cell(1, 3, 3), Cell(5, 3, 3), ref path), "a point flyer fits through");

            agent.body = tall;
            Vector3 lift = tall.Lift(pd);
            Assert.IsFalse(agent.GetPath(Cell(1, 3, 3) + lift, Cell(5, 3, 3) + lift, ref path), "a tall body doesn't");
        }

        [Test]
        public void TwoCellOpeningLetsTheTallBodyThroughAndItsPathIsItsCentre() {
            Build(new Vector3Int(3, 3, 3), new Vector3Int(3, 4, 3));
            agent.body = tall;
            Vector3 lift = tall.Lift(pd);
            var path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Cell(1, 3, 3) + lift, Cell(5, 3, 3) + lift, ref path));

            Assert.AreEqual(Cell(5, 3, 3) + lift, path[path.Count - 1], "the last point is the goal's centre");
            foreach(Vector3 point in path) {
                // Every point is a body's centre: half a cell above a cell on this grid.
                float above = (point.y - origin.y) / pd;
                Assert.AreEqual(0.5f, above - Mathf.Floor(above), 1e-4f, $"{point} is not half a cell up");
            }
        }

        [Test]
        public void TallBodyStartingInsideTheFloorMovesUpToWhereItFits() {
            // The floor: everything below y == 2 is solid.
            Build(new Vector3Int(3, 3, 3), new Vector3Int(3, 4, 3));
            for(int x = 0; x < size; x++) {
                for(int z = 0; z < size; z++) {
                    grids[x][0][z].BlockedArea(true);
                    grids[x][1][z].BlockedArea(true);
                }
            }

            collection.InitializeGrid(grids);
            collection.ComputeIslands().GetAwaiter().GetResult();
            agent.body = tall;
            Vector3 lift = tall.Lift(pd);
            var path = new List<Vector3>();
            // Its centre half a cell above the floor's top cell: its lowest cell would be in the floor.
            Assert.IsTrue(agent.GetPath(Cell(1, 1, 3) + lift, Cell(5, 3, 3) + lift, ref path));
            Assert.AreEqual(Cell(5, 3, 3) + lift, path[path.Count - 1]);
        }

        [Test]
        public void TallBodyStandsOnTheTopCellWithItsHeadAboveTheGrid() {
            // The sky over the grid is open: the top cell is enough for its lowest; a side of the grid is not.
            Build(new Vector3Int(3, 3, 3), new Vector3Int(3, 4, 3));
            agent.body = tall;
            Vector3 lift = tall.Lift(pd);
            var path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Cell(1, 3, 3) + lift, Cell(1, size - 1, 3) + lift, ref path), "up to the top cell");
            Assert.AreEqual(Cell(1, size - 1, 3) + lift, path[path.Count - 1]);
        }

        [Test]
        public void PrunedSegmentsOfATallBodyStayWhereItFits() {
            Build(new Vector3Int(3, 3, 3), new Vector3Int(3, 4, 3));
            agent.body = tall;
            Vector3 lift = tall.Lift(pd);
            var path = new List<Vector3>();
            Assert.IsTrue(agent.GetPath(Cell(1, 1, 1) + lift, Cell(5, 5, 5) + lift, ref path));
            Vector3 from = Cell(1, 1, 1);
            foreach(Vector3 point in path) {
                Vector3 to = point - lift;
                // Its lowest cells and the ones above them: both lines are clear.
                Assert.IsTrue(collection.HasLineOfSight(from, to), $"{from} -> {to}");
                Assert.IsTrue(collection.HasLineOfSight(from + Vector3.up * pd, to + Vector3.up * pd), $"{from} -> {to}, a cell up");
                from = to;
            }
        }
    }
}
