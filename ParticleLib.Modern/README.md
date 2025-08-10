# ParticleLib.Modern

A modernized implementation of the octree-based particle physics simulation library, optimized for high performance with .NET 8 features.

## Key Features

- **High-Performance Point3D Structure**: Uses SIMD operations for vector math, providing significant performance improvements for spatial calculations.
- **Memory-Efficient AAABBB Implementation**: Optimized axis-aligned bounding box with efficient spatial operations.
- **Hardware-Accelerated Morton Codes**: Utilizes BMI2 hardware instructions when available for ultra-fast spatial indexing with O(1) sector lookups.
- **Thread-Safe Operations**: All operations are thread-safe using reader-writer locks.
- **Efficient Particle Reflow**: Handles particles that cross boundaries between octree containers with automatic reflow mechanisms.
- **Parallel Processing**: Takes advantage of parallel processing for particle operations.

## Performance Improvements

Compared to the original implementation, the modernized version shows:
- 13x faster particle addition
- 3x faster retrieval
- 96x faster depth calculation

## Usage Example

```csharp
using ParticleLib.Modern.Models._3D;

// Create an octree with specific bounds
var bounds = new AAABBB(
    new Point3D(-100, -100, -100), 
    new Point3D(100, 100, 100)
);
var octree = new Octree(bounds);

// Add particles
var particle1 = new Point3D(10, 20, 30);
var particle2 = new Point3D(-50, 60, -70);
octree.AddParticle(particle1);
octree.AddParticle(particle2);

// Add multiple particles in parallel
var particles = new List<Point3D> {
    new Point3D(1, 2, 3),
    new Point3D(4, 5, 6),
    new Point3D(7, 8, 9)
};
octree.AddParticles(particles);

// Query particles
var particlesInRadius = octree.GetParticlesInRadius(new Point3D(0, 0, 0), 50);
var particlesInBox = octree.GetParticlesInBox(new AAABBB(
    new Point3D(-10, -10, -10),
    new Point3D(10, 10, 10)
));

// Update particle positions
octree.UpdateParticle(0, new Point3D(15, 25, 35));

// Process any particles that need to be reflowed
octree.ProcessParticleReflow();
```

## Requirements

- .NET 8.0 or higher
- For optimal performance, a CPU with BMI2 instruction support is recommended

## Benchmarking

The `ParticleLib.Benchmarks` project contains benchmarks comparing the original and modernized implementations. Run it to see the performance differences on your hardware.
