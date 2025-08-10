using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace ParticleLib.Modern.Models._3D
{
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct OctreeNode : IEquatable<OctreeNode>
    {
        public readonly ulong MortonCode;
        public readonly byte Depth;
        public readonly AAABBB BoundingBox;
        public readonly byte Octant;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public OctreeNode(ulong morton, byte depth, AAABBB box, byte octant)
        {
            MortonCode = morton;
            Depth = depth;
            BoundingBox = box;
            Octant = octant;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OctreeNode CreateRoot(AAABBB box) => new OctreeNode(0, 0, box, 0);

        // depth-marker bit removed; only shift left then OR octant
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public OctreeNode CreateChild(byte octant)
        {
            var sub = BoundingBox.Split();
            ulong code = (MortonCode << 3) | octant;
            return new OctreeNode(code, (byte)(Depth + 1), sub[octant], octant);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public byte GetChildOctantForPoint(Point3D p) => BoundingBox.GetOctant(p);

        public bool Equals(OctreeNode other) => MortonCode == other.MortonCode && Depth == other.Depth;
        public override bool Equals(object obj) => obj is OctreeNode n && Equals(n);
        public override int GetHashCode() => ((int)MortonCode) ^ ((int)(MortonCode >> 32)) ^ Depth;
        public override string ToString() => $"Node[{MortonCode}, d{Depth}, o{Octant}]";
    }

    [Flags]
    public enum NodeType
    {
        None = 0,
        Internal = 1,
        Leaf = 2
    }
}
