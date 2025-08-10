using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ParticleLib.Modern.Models._3D
{

    /// <summary>
    /// Dictionary key that uniquely identifies a node by
    /// (Morton code, depth) without wasting an extra bit.
    /// </summary>
    internal readonly struct NodeKey : IEquatable<NodeKey>
    {
        public readonly ulong Code;
        public readonly byte Depth;

        public NodeKey(ulong code, byte depth)
        {
            Code = code;
            Depth = depth;
        }

        public bool Equals(NodeKey other) =>
            Code == other.Code && Depth == other.Depth;

        public override bool Equals(object obj) =>
            obj is NodeKey nk && Equals(nk);

        public override int GetHashCode() =>
            ((int)Code) ^ ((int)(Code >> 32)) ^ Depth;
    }
}
