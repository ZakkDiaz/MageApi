using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using Microsoft.Extensions.ObjectPool;

namespace ParticleLib.Modern.Models._3D
{
    /// <summary>
    /// High-performance octree (Morton-free) with per-node locks and pooled leaf lists.
    /// Public surface is unchanged. Internals are simpler and deadlock-safe.
    /// </summary>
    public sealed class Octree : IDisposable
    {
        // ---------- config ----------
        private readonly int _maxParticlesPerLeaf;
        private readonly int _maxDepth;

        // ---------- global state ----------
        private readonly List<NodeEntry> _nodes = new();
        private readonly object _particleArraysLock = new object();
        private readonly ReaderWriterLockSlim _treeLock = new(LockRecursionPolicy.NoRecursion);

        private static readonly ObjectPool<List<int>> _listPool =
            new DefaultObjectPoolProvider { MaximumRetained = 1024 }.Create<List<int>>();

        // particle arrays
        private readonly List<Point3D> _particles = new();
        private readonly List<int> _particleLeaf = new(); // index -> leaf node index
        private readonly List<int> _indexInLeaf = new(); // particleId -> index within its leaf list

        // reflow queue
        private readonly ConcurrentQueue<int> _particlesToReflow = new();
        private const int BulkInsertThreshold = 8192;

        // root (always index 0 in _nodes)
        private readonly NodeEntry _root;

        // ---------- public surface ----------
        public AAABBB Bounds => _root.Node.BoundingBox;

        public int ParticleCount
        {
            get { lock (_particleArraysLock) return _particles.Count; }
        }

        public int NodeCount
        {
            get { _treeLock.EnterReadLock(); try { return _nodes.Count; } finally { _treeLock.ExitReadLock(); } }
        }

        public Octree(AAABBB bounds, int maxParticlesPerLeaf = 10, int maxDepth = 27)
        {
            _maxParticlesPerLeaf = Math.Max(1, maxParticlesPerLeaf);
            _maxDepth = Math.Max(0, maxDepth);

            // Create root as a LEAF with an empty pooled list
            _root = new NodeEntry(OctreeNode.CreateRoot(bounds), _listPool.Get());

            _treeLock.EnterWriteLock();
            try
            {
                _nodes.Add(_root); // index 0
            }
            finally { _treeLock.ExitWriteLock(); }
        }

        // ----------------- public API -----------------

        public int AddParticle(Point3D position)
        {
            if (!Bounds.Contains(position))
                throw new ArgumentOutOfRangeException(nameof(position));

            int idx;
            lock (_particleArraysLock)
            {
                idx = _particles.Count;
                _particles.Add(position);
                _particleLeaf.Add(-1);
                _indexInLeaf.Add(-1);
            }
            InsertParticle(idx, position);
            return idx;
        }


        public int[] AddParticles(IEnumerable<Point3D> positions)
        {
            if (positions is null) return Array.Empty<int>();
            var arr = positions as IList<Point3D> ?? positions.ToList();
            if (arr.Count == 0) return Array.Empty<int>();

            // Small batches: stick with single inserts (keeps contention minimal)
            if (arr.Count < BulkInsertThreshold)
            {
                var outIdx = new int[arr.Count];
                for (int i = 0; i < arr.Count; i++)
                    outIdx[i] = AddParticle(arr[i]);
                ProcessParticleReflow();
                return outIdx;
            }

            // Large batch: partition top-down under a single write lock
            return BulkAddParticles(arr);
        }
        private int[] BulkAddParticles(IList<Point3D> arr)
        {
            int n = arr.Count;
            var outIdx = new int[n];

            // 1) Reserve IDs & write particle positions
            int baseId;
            lock (_particleArraysLock)
            {
                baseId = _particles.Count;
                _particles.Capacity = Math.Max(_particles.Capacity, baseId + n);
                _particleLeaf.Capacity = Math.Max(_particleLeaf.Capacity, baseId + n);
                _indexInLeaf.Capacity = Math.Max(_indexInLeaf.Capacity, baseId + n);

                for (int i = 0; i < n; i++)
                {
                    _particles.Add(arr[i]);
                    _particleLeaf.Add(-1);
                    _indexInLeaf.Add(-1);
                    outIdx[i] = baseId + i;
                }
            }

            // 2) Single-writer build: top-down binning of the new IDs
            _treeLock.EnterWriteLock();
            try
            {
                // Queue of (nodeIdx, binOfParticleIds)
                var q = new Queue<(int node, List<int> bin)>(128);

                // Root bin
                var rootBin = _listPool.Get();
                for (int i = 0; i < n; i++) rootBin.Add(baseId + i);
                q.Enqueue((0, rootBin));

                while (q.Count > 0)
                {
                    var (nodeIdx, bin) = q.Dequeue();
                    var e = _nodes[nodeIdx];

                    if (e.IsLeaf)
                    {
                        if (e.Node.Depth >= _maxDepth || e.Leaf!.Count + bin.Count <= _maxParticlesPerLeaf)
                        {
                            // Append all at once + index bookkeeping
                            var list = e.Leaf!;
                            int baseCount = list.Count;
                            list.EnsureCapacity(baseCount + bin.Count);
                            list.AddRange(bin);
                            for (int i = 0; i < bin.Count; i++)
                            {
                                int pid = bin[i];
                                _particleLeaf[pid] = nodeIdx;
                                _indexInLeaf[pid] = baseCount + i;
                            }

                            bin.Clear(); _listPool.Return(bin);
                        }
                        else
                        {
                            // Split this leaf, partition existing + new bin to children, then continue
                            var parts = e.Leaf!;
                            e.Leaf = null; // becomes internal

                            List<int>?[] childBins = new List<int>?[8];

                            // existing payload
                            for (int i = 0; i < parts.Count; i++)
                            {
                                int pid = parts[i];
                                var p = _particles[pid];
                                byte oct = e.Node.GetChildOctantForPoint(p);
                                (childBins[oct] ??= _listPool.Get()).Add(pid);
                            }
                            parts.Clear(); _listPool.Return(parts);

                            // new payload for this node
                            for (int i = 0; i < bin.Count; i++)
                            {
                                int pid = bin[i];
                                var p = _particles[pid];
                                byte oct = e.Node.GetChildOctantForPoint(p);
                                (childBins[oct] ??= _listPool.Get()).Add(pid);
                            }
                            bin.Clear(); _listPool.Return(bin);

                            for (byte oct = 0; oct < 8; oct++)
                            {
                                var cb = childBins[oct];
                                if (cb is null || cb.Count == 0) continue;

                                int childIdx = EnsureChild_NoLock(nodeIdx, oct);
                                q.Enqueue((childIdx, cb)); // process child in the same loop
                            }
                        }
                    }
                    else
                    {
                        // Internal node: just route the bin to children
                        List<int>?[] childBins = new List<int>?[8];
                        for (int i = 0; i < bin.Count; i++)
                        {
                            int pid = bin[i];
                            var p = _particles[pid];
                            byte oct = e.Node.GetChildOctantForPoint(p);
                            (childBins[oct] ??= _listPool.Get()).Add(pid);
                        }
                        bin.Clear(); _listPool.Return(bin);

                        for (byte oct = 0; oct < 8; oct++)
                        {
                            var cb = childBins[oct];
                            if (cb is null || cb.Count == 0) continue;

                            int childIdx = EnsureChild_NoLock(nodeIdx, oct);
                            q.Enqueue((childIdx, cb));
                        }
                    }
                }
            }
            finally { _treeLock.ExitWriteLock(); }

            return outIdx;
        }



        public void UpdateParticle(int id, Point3D pos)
        {
            if (!Bounds.Contains(pos))
            {
                var x = pos.X;
                var y = pos.Y;
                var z = pos.Z;
                if (pos.X < Bounds.Min.X)
                    x = Bounds.Min.X;
                else if (pos.X > Bounds.Max.X)
                    x = Bounds.Max.X;
                if (pos.Y < Bounds.Min.Y)
                    y = Bounds.Min.Y;
                else if (pos.Y > Bounds.Max.Y)
                    y = Bounds.Max.Y;
                if (pos.Z < Bounds.Min.Z)
                    z = Bounds.Min.Z;
                else if (pos.Z > Bounds.Max.Z)
                    z = Bounds.Max.Z;
                pos = new Point3D(x, y, z);
            }

            // Check if the *current* leaf still contains the new position.
            // If so, we only update coordinates—no reflow needed.
            int leaf = _particleLeaf[id];
            bool needsReflow = true;
            if (leaf >= 0)
            {
                var box = _nodes[leaf].Node.BoundingBox;
                needsReflow = !box.Contains(pos);
            }

            lock (_particleArraysLock)
            {
                _particles[id] = pos;
            }

            if (needsReflow) _particlesToReflow.Enqueue(id);
        }


        public void RemoveParticle(int id)
        {
            int leaf = _particleLeaf[id];
            if (leaf >= 0)
            {
                var e = _nodes[leaf];
                e.Lock.Enter();
                try
                {
                    if (e.Leaf is { Count: > 0 })
                        LeafRemoveUnlocked(leaf, id);
                }
                finally { e.Lock.Exit(); }
            }

            lock (_particleArraysLock)
            {
                _particles[id] = new Point3D(float.NaN, float.NaN, float.NaN);
            }
        }


        public Point3D[] GetAllParticles()
        {
            lock (_particleArraysLock)
                return _particles.ToArray();
        }

        public int[] GetParticlesInRadius(Point3D center, float radius)
        {
            var result = new List<int>();

            _treeLock.EnterReadLock();
            try
            {
                var aabb = new AAABBB(
                    new Point3D(center.X - radius, center.Y - radius, center.Z - radius),
                    new Point3D(center.X + radius, center.Y + radius, center.Z + radius));

                var stack = new Stack<int>();
                stack.Push(0);

                while (stack.Count > 0)
                {
                    int idx = stack.Pop();
                    var e = _nodes[idx];
                    if (!e.Node.BoundingBox.Intersects(aabb)) continue;

                    if (e.IsLeaf)
                    {
                        // Hold the node lock while giving the list to SIMD helper
                        e.Lock.Enter();
                        try
                        {
                            if (e.Leaf is { Count: > 0 })
                                SimdSearch.AppendHitsInRadius(center, radius, e.Leaf, _particles, result);
                        }
                        finally { e.Lock.Exit(); }
                    }
                    else
                    {
                        for (int o = 0; o < 8; o++)
                        {
                            int c = e.Child[o];
                            if (c >= 0) stack.Push(c);
                        }
                    }
                }
            }
            finally { _treeLock.ExitReadLock(); }

            return result.ToArray();
        }

        public int[] GetParticlesInBox(AAABBB box)
        {
            var result = new List<int>();

            _treeLock.EnterReadLock();
            try
            {
                var stack = new Stack<int>();
                stack.Push(0);

                while (stack.Count > 0)
                {
                    int idx = stack.Pop();
                    var e = _nodes[idx];
                    if (!e.Node.BoundingBox.Intersects(box)) continue;

                    if (e.IsLeaf)
                    {
                        e.Lock.Enter();
                        try
                        {
                            if (e.Leaf is { Count: > 0 })
                            {
                                foreach (int pi in e.Leaf)
                                {
                                    var p = _particles[pi];
                                    if (!float.IsNaN(p.X) && box.Contains(p))
                                        result.Add(pi);
                                }
                            }
                        }
                        finally { e.Lock.Exit(); }
                    }
                    else
                    {
                        for (int o = 0; o < 8; o++)
                        {
                            int c = e.Child[o];
                            if (c >= 0) stack.Push(c);
                        }
                    }
                }
            }
            finally { _treeLock.ExitReadLock(); }

            return result.ToArray();
        }

        public int GetDepth()
        {
            _treeLock.EnterReadLock();
            try { return _nodes.Count == 0 ? 0 : _nodes.Max(n => n.Node.Depth); }
            finally { _treeLock.ExitReadLock(); }
        }

        public AAABBB[] GetBoxCloud()
        {
            _treeLock.EnterReadLock();
            try
            {
                var arr = new AAABBB[_nodes.Count];
                for (int i = 0; i < _nodes.Count; i++) arr[i] = _nodes[i].Node.BoundingBox;
                return arr;
            }
            finally { _treeLock.ExitReadLock(); }
        }

        public void ProcessParticleReflow()
        {
            while (_particlesToReflow.TryDequeue(out int id))
            {
                if (float.IsNaN(_particles[id].X)) continue;

                int oldLeaf = _particleLeaf[id];
                if (oldLeaf >= 0)
                {
                    var e = _nodes[oldLeaf];
                    e.Lock.Enter();
                    try
                    {
                        if (e.Leaf is { Count: > 0 })
                            LeafRemoveUnlocked(oldLeaf, id);
                    }
                    finally { e.Lock.Exit(); }
                }

                InsertParticle(id, _particles[id]);
            }
        }


        public void Clear()
        {
            _treeLock.EnterWriteLock();
            try
            {
                foreach (var e in _nodes)
                {
                    if (e.Leaf != null)
                    {
                        e.Leaf.Clear();
                        _listPool.Return(e.Leaf);
                    }
                    for (int i = 0; i < 8; i++) e.Child[i] = -1;
                    e.Leaf = null;
                }

                lock (_particleArraysLock)
                {
                    _particles.Clear();
                    _particleLeaf.Clear();
                    _indexInLeaf.Clear();
                }
                _particlesToReflow.Clear();

                _nodes.Clear();
                _root.Leaf = _listPool.Get();
                _nodes.Add(_root);
            }
            finally { _treeLock.ExitWriteLock(); }
        }


        public void Dispose()
        {
            _treeLock.Dispose();
        }

        // ----------------- internals -----------------

        private void InsertParticle(int id, Point3D pos)
        {
            int current = 0;

            while (true)
            {
                _treeLock.EnterUpgradeableReadLock();
                try
                {
                    var entry = _nodes[current];

                    if (entry.IsLeaf)
                    {
                        // Try fast-path insert into this leaf
                        entry.Lock.Enter();
                        try
                        {
                            if (!entry.IsLeaf)
                            {
                                continue; // flipped while we waited
                            }

                            if (entry.Node.Depth >= _maxDepth || entry.Leaf!.Count < _maxParticlesPerLeaf)
                            {
                                LeafAddUnlocked(current, id);
                                return;
                            }
                        }
                        finally { entry.Lock.Exit(); }

                        // Need to split this leaf
                        _treeLock.EnterWriteLock();
                        try
                        {
                            entry = _nodes[current];
                            entry.Lock.Enter();
                            try
                            {
                                if (!entry.IsLeaf)
                                {
                                    // already split
                                }
                                else if (entry.Node.Depth >= _maxDepth)
                                {
                                    LeafAddUnlocked(current, id);
                                    return;
                                }
                                else
                                {
                                    // Split: partition payload to 8 bins
                                    var parts = entry.Leaf!;
                                    entry.Leaf = null; // becomes internal

                                    List<int>?[] bins = new List<int>?[8];

                                    foreach (int pid in parts)
                                    {
                                        Point3D p;
                                        lock (_particleArraysLock) p = _particles[pid];
                                        byte oct = entry.Node.GetChildOctantForPoint(p);
                                        var bin = bins[oct] ??= _listPool.Get();
                                        bin.Add(pid);
                                    }

                                    parts.Clear();
                                    _listPool.Return(parts);

                                    // Create/ensure children and append payload; set indices
                                    for (byte oct = 0; oct < 8; oct++)
                                    {
                                        var bin = bins[oct];
                                        if (bin is null || bin.Count == 0) continue;

                                        int childIdx = EnsureChild_NoLock(current, oct);
                                        var child = _nodes[childIdx];

                                        // Append + index bookkeeping
                                        int baseCount = child.Leaf!.Count;
                                        child.Leaf.AddRange(bin);
                                        for (int i = 0; i < bin.Count; i++)
                                        {
                                            int pid = bin[i];
                                            _particleLeaf[pid] = childIdx;
                                            _indexInLeaf[pid] = baseCount + i;
                                        }

                                        bin.Clear();
                                        _listPool.Return(bin);
                                    }
                                }
                            }
                            finally { entry.Lock.Exit(); }

                            // Descend to the correct child for 'pos'
                            var parentIdx = current;
                            var parent = _nodes[parentIdx];
                            byte go = parent.Node.GetChildOctantForPoint(pos);
                            int next = parent.Child[go];
                            if (next < 0) next = EnsureChild_NoLock(parentIdx, go);
                            current = next;
                        }
                        finally { _treeLock.ExitWriteLock(); }
                    }
                    else
                    {
                        // Internal: descend
                        byte oct = entry.Node.GetChildOctantForPoint(pos);
                        int child = entry.Child[oct];

                        if (child >= 0)
                        {
                            current = child;
                            continue;
                        }

                        _treeLock.EnterWriteLock();
                        try
                        {
                            child = entry.Child[oct];
                            if (child < 0)
                                child = EnsureChild_NoLock(current, oct);
                            current = child;
                        }
                        finally { _treeLock.ExitWriteLock(); }
                    }
                }
                finally
                {
                    _treeLock.ExitUpgradeableReadLock();
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void LeafAddUnlocked(int leafIdx, int pid)
        {
            var e = _nodes[leafIdx];
            var list = e.Leaf!;
            list.Add(pid);
            _particleLeaf[pid] = leafIdx;
            _indexInLeaf[pid] = list.Count - 1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void LeafRemoveUnlocked(int leafIdx, int pid)
        {
            var e = _nodes[leafIdx];
            var list = e.Leaf!;
            int idx = _indexInLeaf[pid];

            if (idx < 0 || idx >= list.Count)
            {
                // Fallback if index is stale (shouldn't happen, but be defensive)
                if (list.Remove(pid))
                {
                    _indexInLeaf[pid] = -1;
                    _particleLeaf[pid] = -1;
                }
                return;
            }

            int last = list.Count - 1;
            int lastPid = list[last];
            list[idx] = lastPid;
            list.RemoveAt(last);

            _indexInLeaf[lastPid] = idx;
            _indexInLeaf[pid] = -1;
            _particleLeaf[pid] = -1;
        }


        // Create missing child under TREE WRITE lock; returns the child index.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int EnsureChild_NoLock(int parentIdx, byte oct)
        {
            var p = _nodes[parentIdx];
            int existing = p.Child[oct];
            if (existing >= 0) return existing;

            var childNode = p.Node.CreateChild(oct);
            var entry = new NodeEntry(childNode, _listPool.Get()); // child starts as leaf
            int idx = _nodes.Count;
            _nodes.Add(entry);
            p.Child[oct] = idx;
            return idx;
        }


        // Quantize both points into the same depth cell and compare integer bins.
        // This matches the implicit mid-point splitting of the AAABBB.GetOctant path.
        private bool IsInSameLeaf(Point3D a, Point3D b)
        {
            int depth = Math.Min(_maxDepth, 30); // 2^30 bins still safe for float range here
            if (depth == 0) return true;

            var (ax, ay, az) = Quantize(a, depth);
            var (bx, by, bz) = Quantize(b, depth);
            return ax == bx && ay == by && az == bz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private (int x, int y, int z) Quantize(Point3D p, int depth)
        {
            int scale = 1 << depth; // depth <= 30 guarded above
            var min = Bounds.Min; var max = Bounds.Max;

            double nx = (p.X - min.X) / Math.Max(1e-30, (double)(max.X - min.X));
            double ny = (p.Y - min.Y) / Math.Max(1e-30, (double)(max.Y - min.Y));
            double nz = (p.Z - min.Z) / Math.Max(1e-30, (double)(max.Z - min.Z));

            // Clamp to [0, 1) so the max value lands in the last bin
            nx = nx <= 0 ? 0 : (nx >= 1 ? BitDecrementOne() : nx);
            ny = ny <= 0 ? 0 : (ny >= 1 ? BitDecrementOne() : ny);
            nz = nz <= 0 ? 0 : (nz >= 1 ? BitDecrementOne() : nz);

            int ix = (int)(nx * scale);
            int iy = (int)(ny * scale);
            int iz = (int)(nz * scale);
            if (ix >= scale) ix = scale - 1; // paranoia
            if (iy >= scale) iy = scale - 1;
            if (iz >= scale) iz = scale - 1;
            return (ix, iy, iz);

            static double BitDecrementOne() => BitConverter.Int64BitsToDouble(BitConverter.DoubleToInt64Bits(1.0) - 1);
        }

        // ------------- storage for each node (internal) -------------
        private sealed class NodeEntry
        {
            public OctreeNode Node;          // bbox + depth (Morton ignored)
            public readonly int[] Child;     // child indices; -1 if missing
            public List<int>? Leaf;          // non-null => leaf payload
            public readonly NodeSpinLock Lock = new NodeSpinLock();

            public bool IsLeaf => Leaf != null;

            public NodeEntry(OctreeNode node, List<int>? leaf)
            {
                Node = node;
                Leaf = leaf;
                Child = new int[8];
                for (int i = 0; i < 8; i++) Child[i] = -1;
            }
        }
    }

    // ---------- tiny spin lock for per-node writes ----------
    // Reference type to avoid value-copy bugs from list indexers.
    internal sealed class NodeSpinLock
    {
        private int _state;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Enter()
        {
            var s = new SpinWait();
            while (Interlocked.CompareExchange(ref _state, 1, 0) != 0) s.SpinOnce();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Exit() { Volatile.Write(ref _state, 0); }
    }
}
