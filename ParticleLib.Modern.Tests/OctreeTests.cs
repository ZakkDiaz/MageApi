using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Xunit;
using ParticleLib.Modern.Models._3D;

namespace ParticleLib.Modern.Tests
{
    public class OctreeTests
    {
        [Fact]
        public void Constructor_CreatesValidOctree()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            
            // Act
            var octree = new Octree(bounds);
            
            // Assert
            Assert.Equal(bounds, octree.Bounds);
            Assert.Equal(0, octree.ParticleCount);
            Assert.Equal(1, octree.NodeCount); // Just the root node
        }

        [Fact]
        public void AddParticle_SingleParticle_AddsCorrectly()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds);
            var position = new Point3D(10, 20, 30);
            
            // Act
            int index = octree.AddParticle(position);
            
            // Assert
            Assert.Equal(0, index);
            Assert.Equal(1, octree.ParticleCount);
            Assert.Contains(position, octree.GetAllParticles());
        }

        [Fact]
        public void AddParticles_MultipleParticles_AddsAllCorrectly()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds);
            var positions = new List<Point3D>
            {
                new Point3D(10, 20, 30),
                new Point3D(-10, -20, -30),
                new Point3D(50, 60, 70),
                new Point3D(-50, -60, -70)
            };
            
            // Act
            int[] indices = octree.AddParticles(positions);
            
            // Assert
            Assert.Equal(4, indices.Length);
            Assert.Equal(4, octree.ParticleCount);
            
            var allParticles = octree.GetAllParticles();
            foreach (var position in positions)
            {
                Assert.Contains(position, allParticles);
            }
        }

        [Fact]
        public void GetParticlesInRadius_ReturnsCorrectParticles()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds);
            
            // Add particles in a grid pattern
            var positions = new List<Point3D>();
            for (int x = -50; x <= 50; x += 10)
            {
                for (int y = -50; y <= 50; y += 10)
                {
                    for (int z = -50; z <= 50; z += 10)
                    {
                        positions.Add(new Point3D(x, y, z));
                    }
                }
            }
            
            octree.AddParticles(positions);
            
            // Act
            var center = new Point3D(0, 0, 0);
            var particlesInRadius = octree.GetParticlesInRadius(center, 15);
            
            // Assert
            // Should include (0,0,0) and all particles at distance 10 (±10,±10,±10)
            Assert.Equal(19, particlesInRadius.Length);
            
            // Verify all particles are within the radius
            foreach (var index in particlesInRadius)
            {
                var position = octree.GetAllParticles()[index];
                Assert.True(Point3D.Distance(center, position) <= 15);
            }
        }

        [Fact]
        public void GetParticlesInBox_ReturnsCorrectParticles()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds);
            
            // Add particles in a grid pattern
            var positions = new List<Point3D>();
            for (int x = -50; x <= 50; x += 10)
            {
                for (int y = -50; y <= 50; y += 10)
                {
                    for (int z = -50; z <= 50; z += 10)
                    {
                        positions.Add(new Point3D(x, y, z));
                    }
                }
            }
            
            octree.AddParticles(positions);
            
            // Act
            var queryBox = new AAABBB(new Point3D(-25, -25, -25), new Point3D(25, 25, 25));
            var particlesInBox = octree.GetParticlesInBox(queryBox);
            
            // Assert
            // Should include all particles with coordinates between -20 and 20
            Assert.Equal(125, particlesInBox.Length); // 5x5x5 grid of points
            
            // Verify all particles are within the box
            foreach (var index in particlesInBox)
            {
                var position = octree.GetAllParticles()[index];
                Assert.True(queryBox.Contains(position));
            }
        }

        [Fact]
        public void UpdateParticle_MovesParticleCorrectly()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds);
            var initialPosition = new Point3D(10, 20, 30);
            var newPosition = new Point3D(50, 60, 70);
            
            // Add a particle
            int index = octree.AddParticle(initialPosition);
            
            // Act
            octree.UpdateParticle(index, newPosition);
            
            // Process any particle reflow
            octree.ProcessParticleReflow();
            
            // Assert
            var allParticles = octree.GetAllParticles();
            Assert.Equal(newPosition, allParticles[index]);
            
            // The particle should be found at its new position
            var particlesAtNewPos = octree.GetParticlesInRadius(newPosition, 1);
            Assert.Contains(index, particlesAtNewPos);
            
            // The particle should not be found at its old position
            var particlesAtOldPos = octree.GetParticlesInRadius(initialPosition, 1);
            Assert.DoesNotContain(index, particlesAtOldPos);
        }

        [Fact]
        public void RemoveParticle_RemovesCorrectly()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds);
            var position = new Point3D(10, 20, 30);
            
            // Add a particle
            int index = octree.AddParticle(position);
            
            // Act
            octree.RemoveParticle(index);
            
            // Assert
            // The particle count should still be 1 (we don't actually remove from the list)
            Assert.Equal(1, octree.ParticleCount);
            
            // But the particle should not be found in queries
            var particlesAtPos = octree.GetParticlesInRadius(position, 1);
            Assert.Empty(particlesAtPos);
        }

        [Fact]
        public void GetDepth_ReturnsCorrectDepth()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds, maxParticlesPerLeaf: 1, maxDepth: 8);
            
            // Add particles that will force the tree to split
            for (int i = 0; i < 10; i++)
            {
                octree.AddParticle(new Point3D(0, 0, i * 0.1f));
            }
            
            // Act
            int depth = octree.GetDepth();
            
            // Assert
            Assert.True(depth > 1, $"Depth should be greater than 1, got {depth}");
        }

        [Fact]
        public void Clear_RemovesAllParticles()
        {
            // Arrange
            var bounds = new AAABBB(new Point3D(-100, -100, -100), new Point3D(100, 100, 100));
            var octree = new Octree(bounds);
            
            // Add some particles
            for (int i = 0; i < 10; i++)
            {
                octree.AddParticle(new Point3D(i * 10, i * 10, i * 10));
            }
            
            // Act
            octree.Clear();
            
            // Assert
            Assert.Equal(0, octree.ParticleCount);
            Assert.Equal(1, octree.NodeCount); // Just the root node
            Assert.Empty(octree.GetAllParticles());
        }
    }
}
