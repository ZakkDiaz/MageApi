using System;
using System.Collections.Generic;
using System.Threading;
using MageUniverse.Physics;
using ParticleLib.Modern.Models._3D;

namespace MageUniverse;

public class Universe : IUniverse
{
    private readonly List<Particle> _particles = new();
    private readonly Timer _timer;
    private readonly Octree _octree;
    private const float TargetInterval = 1f / 120f; // Target 120 updates per second
    private DateTime _lastUpdateTime;

    // physics constants
    const float minDistance = 1f;
    const float maxForce = 1000f;
    const float neighborRadius = 100f;

    public Universe(AAABBB bounds)
    {
        _octree = new Octree(bounds);
        _lastUpdateTime = DateTime.UtcNow;
        _timer = new Timer(TimerCallback, null, TimeSpan.Zero, TimeSpan.FromSeconds(TargetInterval));
    }

    public void Clear()
    {
        _particles.Clear();
        _octree.Clear();
    }

    private void TimerCallback(object? state)
    {
        var now = DateTime.UtcNow;
        var deltaTime = (float)(now - _lastUpdateTime).TotalSeconds;
        _lastUpdateTime = now;
        deltaTime /= 2;
        TimeStep(deltaTime);
    }

    public void AddParticle(Particle particle)
    {
        _particles.Add(particle);
        particle.OctreeIndex = _octree.AddParticle(new Point3D(particle.LocationX, particle.LocationY, particle.LocationZ));
    }

    public void AddParticles(IEnumerable<Particle> newParticles)
    {
        foreach (var particle in newParticles)
        {
            AddParticle(particle);
        }
    }

    public List<Particle> GetCurrentState()
    {
        return new List<Particle>(_particles);
    }

    public void TimeStep(float deltaTime)
    {
        // compute gravitational forces
        for (int i = 0; i < _particles.Count; i++)
        {
            var particle = _particles[i];
            var neighbors = _octree.GetParticlesInRadius(
                new Point3D(particle.LocationX, particle.LocationY, particle.LocationZ),
                neighborRadius);

            foreach (var neighborId in neighbors)
            {
                if (neighborId <= i) continue;

                var particle1 = _particles[neighborId];
                float dx = particle1.LocationX - particle.LocationX;
                float dy = particle1.LocationY - particle.LocationY;
                float dz = particle1.LocationZ - particle.LocationZ;
                float distanceSq = dx * dx + dy * dy + dz * dz;
                float distance = MathF.Sqrt(distanceSq);
                if (distance < minDistance) distance = minDistance;

                float force = particle.Mass * particle1.Mass / distanceSq;
                if (force > maxForce) force = maxForce;
                float invDist = 1f / distance;
                float ax = dx * invDist;
                float ay = dy * invDist;
                float az = dz * invDist;

                particle.VelocityX += force * ax / particle.Mass * deltaTime;
                particle.VelocityY += force * ay / particle.Mass * deltaTime;
                particle.VelocityZ += force * az / particle.Mass * deltaTime;

                particle1.VelocityX -= force * ax / particle1.Mass * deltaTime;
                particle1.VelocityY -= force * ay / particle1.Mass * deltaTime;
                particle1.VelocityZ -= force * az / particle1.Mass * deltaTime;
            }
        }

        var bounds = _octree.Bounds;
        foreach (var particle in _particles)
        {
            particle.LocationX += particle.VelocityX * deltaTime;
            particle.LocationY += particle.VelocityY * deltaTime;
            particle.LocationZ += particle.VelocityZ * deltaTime;

            particle.LocationX = Math.Clamp(particle.LocationX, bounds.Min.X, bounds.Max.X);
            particle.LocationY = Math.Clamp(particle.LocationY, bounds.Min.Y, bounds.Max.Y);
            particle.LocationZ = Math.Clamp(particle.LocationZ, bounds.Min.Z, bounds.Max.Z);

            _octree.UpdateParticle(particle.OctreeIndex, new Point3D(particle.LocationX, particle.LocationY, particle.LocationZ));
        }

        _octree.ProcessParticleReflow();
    }

    // Method to stop the timer if needed
    public void Stop()
    {
        _timer?.Change(Timeout.Infinite, Timeout.Infinite);
    }
}

