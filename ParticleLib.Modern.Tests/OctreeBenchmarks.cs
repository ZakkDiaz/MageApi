using System;
using System.Collections.Generic;
using System.Linq;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using ParticleLib.Models._3D;
using ModernOctree = ParticleLib.Modern.Models._3D.Octree;
using ModernPoint3D = ParticleLib.Modern.Models._3D.Point3D;
using ModernAAABBB = ParticleLib.Modern.Models._3D.AAABBB;
using OriginalOctree = ParticleLib.Models._3D.Octree;
using OriginalPoint3D = ParticleLib.Models._3D.Point3D;

namespace ParticleLib.Modern.Tests
{
    [MemoryDiagnoser]
    public class OctreeBenchmarks
    {
        private readonly Random _random = new Random(42); // Fixed seed for reproducibility
        private List<OriginalPoint3D> _originalPoints;
        private List<ModernPoint3D> _modernPoints;
        private OriginalOctree _originalOctree;
        private ModernOctree _modernOctree;

        [Params(100, 1000, 10000)]
        public int ParticleCount { get; set; }

        [GlobalSetup]
        public void Setup()
        {
            // Generate random points for both implementations
            _originalPoints = new List<OriginalPoint3D>();
            _modernPoints = new List<ModernPoint3D>();

            for (int i = 0; i < ParticleCount; i++)
            {
                float x = (float)(_random.NextDouble() * 200 - 100);
                float y = (float)(_random.NextDouble() * 200 - 100);
                float z = (float)(_random.NextDouble() * 200 - 100);

                _originalPoints.Add(new OriginalPoint3D(x, y, z));
                _modernPoints.Add(new ModernPoint3D(x, y, z));
            }

            // Create octrees
            _originalOctree = new OriginalOctree(
                new OriginalPoint3D(-100, -100, -100),
                new OriginalPoint3D(100, 100, 100)
            );

            _modernOctree = new ModernOctree(
                new ModernAAABBB(
                    new ModernPoint3D(-100, -100, -100),
                    new ModernPoint3D(100, 100, 100)
                )
            );
        }

        [Benchmark(Baseline = true)]
        public void OriginalOctree_Construction()
        {
            var octree = new OriginalOctree(
                new OriginalPoint3D(-100, -100, -100),
                new OriginalPoint3D(100, 100, 100)
            );
        }

        [Benchmark]
        public void ModernOctree_Construction()
        {
            var octree = new ModernOctree(
                new ModernAAABBB(
                    new ModernPoint3D(-100, -100, -100),
                    new ModernPoint3D(100, 100, 100)
                )
            );
        }

        [Benchmark]
        public void OriginalOctree_AddParticles()
        {
            var octree = new OriginalOctree(
                new OriginalPoint3D(-100, -100, -100),
                new OriginalPoint3D(100, 100, 100)
            );

            foreach (var point in _originalPoints)
            {
                octree.Add(point.X, point.Y, point.Z);
            }
        }

        [Benchmark]
        public void ModernOctree_AddParticles()
        {
            var octree = new ModernOctree(
                new ModernAAABBB(
                    new ModernPoint3D(-100, -100, -100),
                    new ModernPoint3D(100, 100, 100)
                )
            );

            octree.AddParticles(_modernPoints);
        }

        [Benchmark]
        public void OriginalOctree_GetPointCloud()
        {
            var points = _originalOctree.GetPointCloud();
        }

        [Benchmark]
        public void ModernOctree_GetAllParticles()
        {
            var points = _modernOctree.GetAllParticles();
        }

        [Benchmark]
        public void OriginalOctree_DepthCalculation()
        {
            var depth = _originalOctree.Depth();
        }

        [Benchmark]
        public void ModernOctree_DepthCalculation()
        {
            var depth = _modernOctree.GetDepth();
        }

        [Benchmark]
        public void OriginalOctree_GetBoxCloud()
        {
            var boxes = _originalOctree.GetBoxCloud();
        }

        [Benchmark]
        public void ModernOctree_GetBoxCloud()
        {
            var boxes = _modernOctree.GetBoxCloud();
        }
    }
}
