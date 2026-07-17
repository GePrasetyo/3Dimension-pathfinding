using System;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    [Serializable]
    public class GridNode {
        public Vector3Int coords;
        public Vector3 worldPosition;
        public bool staticInvalid;

        // Refcount so overlapping obstacles releasing their cells can't unblock
        // cells still covered by another obstacle or by static geometry.
        private int dynamicBlocks;

        public bool invalid => staticInvalid || dynamicBlocks > 0;

        /// <summary>
        /// Static blocking only (bake, runtime generation, revalidation).
        /// Dynamic obstacles must use AddDynamicBlock/RemoveDynamicBlock instead.
        /// </summary>
        public void BlockedArea(bool flag) {
            staticInvalid = flag;
        }

        public void AddDynamicBlock() {
            dynamicBlocks++;
        }

        public void RemoveDynamicBlock() {
            if(dynamicBlocks > 0) {
                dynamicBlocks--;
            }
        }
    }
}
