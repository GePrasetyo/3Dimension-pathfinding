using System;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// An agent category as a number, like a layer: a GridVolume and the agents that
    /// should use its grid share one. Names are editor-only (Project Settings >
    /// Pathfinding Agent Types), so builds carry just the byte.
    /// </summary>
    [Serializable]
    public struct PathAgentType : IEquatable<PathAgentType> {
        [SerializeField] private byte value;

        public PathAgentType(byte value) {
            this.value = value;
        }

        public byte Value => value;

        public bool Equals(PathAgentType other) => value == other.value;
        public override bool Equals(object obj) => obj is PathAgentType other && Equals(other);
        public override int GetHashCode() => value;
        public override string ToString() => value.ToString();

        public static bool operator ==(PathAgentType a, PathAgentType b) => a.value == b.value;
        public static bool operator !=(PathAgentType a, PathAgentType b) => a.value != b.value;
        public static implicit operator PathAgentType(byte value) => new PathAgentType(value);
    }
}
