using System.Diagnostics;
using NUnit.Framework;
using UnityEngine;

namespace Majinfwork.Pathfinding.Tests {
    /// <summary>
    /// The island maps' fill (a Burst job): 26-connected regions, and the dynamic map's background refill driven by
    /// GridLocator.TickIslandMaintenance, swapped in only when done and trusted only while no obstacle changed.
    /// </summary>
    public class IslandMapTests {
        private const int size = 7;
        private const float pd = 1f;

        private GridCollection collection;
        private GridNode[][][] grids;

        /// <summary>A size^3 grid, open except a static wall plane at x == 3 with one opening at (3,6,6).</summary>
        [SetUp]
        public void SetUp() {
            collection = Build(size, (x, y, z) => x == 3 && !(y == 6 && z == 6), out grids);
            collection.ComputeIslands().GetAwaiter().GetResult();
            GridLocator.Activate(collection);
        }

        [TearDown]
        public void TearDown() {
            GridLocator.Remove(collection);
        }

        [Test]
        public void CornerTouchingCellsShareAnIsland() {
            // 4^3, everything blocked but (0,0,0), (1,1,1) (a corner apart) and (3,3,3) (two cells off).
            GridCollection corners = Build(4, (x, y, z) => !((x == 0 && y == 0 && z == 0) || (x == 1 && y == 1 && z == 1) || (x == 3 && y == 3 && z == 3)), out GridNode[][][] cells);
            try {
                corners.ComputeIslands().GetAwaiter().GetResult();
                Assert.IsTrue(corners.AreConnected(cells[0][0][0], cells[1][1][1]), "a diagonal step connects, as searches move");
                Assert.IsFalse(corners.AreConnected(cells[0][0][0], cells[3][3][3]));
            }
            finally {
                corners.Dispose();
            }
        }

        [Test]
        public void BackgroundRefillSwapsInOnceDone() {
            grids[3][6][6].AddDynamicBlock();
            GridLocator.BumpObstacleVersion();
            Assert.IsFalse(collection.HasFreshDynamicIslands());

            Assert.IsTrue(TickUntilFresh(), "the refill should finish and swap in");
            Assert.IsFalse(collection.AreConnected(grids[1][3][3], grids[5][3][3]), "the fresh map knows the opening is shut");

            grids[3][6][6].RemoveDynamicBlock();
            GridLocator.BumpObstacleVersion();
            Assert.IsTrue(collection.AreConnected(grids[1][3][3], grids[5][3][3]), "stale map: back to the static one, which is open");
        }

        [Test]
        public void RefillOverChangedObstaclesIsNeverTrusted() {
            grids[3][6][6].AddDynamicBlock();
            GridLocator.BumpObstacleVersion();
            GridLocator.TickIslandMaintenance();

            // Obstacles change while the fill runs: what it filled is already out of date.
            grids[3][6][6].RemoveDynamicBlock();
            GridLocator.BumpObstacleVersion();
            var clock = Stopwatch.StartNew();
            while(clock.ElapsedMilliseconds < 300) {
                GridLocator.TickIslandMaintenance();
                System.Threading.Thread.Sleep(1);
            }

            Assert.IsFalse(collection.HasFreshDynamicIslands());
            Assert.IsTrue(collection.AreConnected(grids[1][3][3], grids[5][3][3]));
        }

        private bool TickUntilFresh() {
            var clock = Stopwatch.StartNew();
            while(clock.ElapsedMilliseconds < 2000) {
                GridLocator.TickIslandMaintenance();
                if(collection.HasFreshDynamicIslands()) {
                    return true;
                }

                System.Threading.Thread.Sleep(1);
            }

            return false;
        }

        private static GridCollection Build(int cells, System.Func<int, int, int, bool> blocked, out GridNode[][][] nodes) {
            Vector3 origin = new Vector3(-cells, -cells, -cells) / 2f * pd;
            var grid = new GridCollection(cells, cells, cells, pd, Vector3.zero, new PathAgentType(0), 0);
            nodes = new GridNode[cells][][];
            for(int x = 0; x < cells; x++) {
                nodes[x] = new GridNode[cells][];
                for(int y = 0; y < cells; y++) {
                    nodes[x][y] = new GridNode[cells];
                    for(int z = 0; z < cells; z++) {
                        var node = new GridNode();
                        node.coords = new Vector3Int(x, y, z);
                        node.worldPosition = origin + new Vector3(x, y, z) * pd;
                        node.BlockedArea(blocked(x, y, z));
                        nodes[x][y][z] = node;
                    }
                }
            }

            grid.InitializeGrid(nodes);
            return grid;
        }
    }
}
