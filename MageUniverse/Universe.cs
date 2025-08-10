using System;
using System.Collections.Generic;
using System.Numerics;
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
    private const float MaxDeltaTime = TargetInterval * 4f; // clamp to avoid large steps
    private DateTime _lastUpdateTime;
    private int _isUpdating; // reentrancy guard for TimerCallback

    // physics constants
    const float minDistance = 1f;
    const float theta = 0.5f; // Barnes-Hut opening angle
    const float farFieldPull = 0.01f;
    const float gravitationalConstant = 1f;

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
        if (Interlocked.Exchange(ref _isUpdating, 1) == 1)
            return; // skip if an update is already in progress
        try
        {
            var now = DateTime.UtcNow;
            var deltaTime = (float)(now - _lastUpdateTime).TotalSeconds;
            _lastUpdateTime = now;
            if (deltaTime > MaxDeltaTime) deltaTime = MaxDeltaTime;
            TimeStep(deltaTime);
        }
        finally
        {
            Volatile.Write(ref _isUpdating, 0);
        }
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
        HandleCollisions();
        _octree.ProcessParticleReflow();
        _octree.RecalculateMasses(id => _particles[id].Mass);

        for (int i = 0; i < _particles.Count; i++)
        {
            var particle = _particles[i];
            var acc = _octree.ComputeAcceleration(
                new Point3D(particle.LocationX, particle.LocationY, particle.LocationZ),
                particle.OctreeIndex,
                id => _particles[id].Mass,
                theta,
                minDistance,
                farFieldPull,
                gravitationalConstant);

            particle.VelocityX += acc.X * deltaTime;
            particle.VelocityY += acc.Y * deltaTime;
            particle.VelocityZ += acc.Z * deltaTime;

            if (!float.IsFinite(particle.VelocityX)) particle.VelocityX = 0f;
            if (!float.IsFinite(particle.VelocityY)) particle.VelocityY = 0f;
            if (!float.IsFinite(particle.VelocityZ)) particle.VelocityZ = 0f;
        }

        var bounds = _octree.Bounds;
        foreach (var particle in _particles)
        {
            particle.LocationX += particle.VelocityX * deltaTime;
            particle.LocationY += particle.VelocityY * deltaTime;
            particle.LocationZ += particle.VelocityZ * deltaTime;

            float x = float.IsFinite(particle.LocationX) ? Math.Clamp(particle.LocationX, bounds.Min.X, bounds.Max.X) : bounds.Center.X;
            float y = float.IsFinite(particle.LocationY) ? Math.Clamp(particle.LocationY, bounds.Min.Y, bounds.Max.Y) : bounds.Center.Y;
            float z = float.IsFinite(particle.LocationZ) ? Math.Clamp(particle.LocationZ, bounds.Min.Z, bounds.Max.Z) : bounds.Center.Z;

            particle.LocationX = x;
            particle.LocationY = y;
            particle.LocationZ = z;

            _octree.UpdateParticle(particle.OctreeIndex, new Point3D(x, y, z));
        }

        _octree.ProcessParticleReflow();
    }

    private void HandleCollisions()
    {
        _octree.ForEachLeaf(indices =>
        {
            for (int i = 0; i < indices.Count; i++)
            {
                var a = _particles[indices[i]];
                for (int j = i + 1; j < indices.Count; j++)
                {
                    var b = _particles[indices[j]];
                    float minDist = a.Radius + b.Radius;
                    var posA = new Vector3(a.LocationX, a.LocationY, a.LocationZ);
                    var posB = new Vector3(b.LocationX, b.LocationY, b.LocationZ);
                    var delta = posB - posA;
                    float dist = delta.Length();
                    if (dist < minDist)
                    {
                        var normal = dist > 1e-5f ? delta / dist : Vector3.UnitX;
                        float penetration = minDist - dist;
                        float totalMass = a.Mass + b.Mass;
                        posA -= normal * (penetration * (b.Mass / totalMass));
                        posB += normal * (penetration * (a.Mass / totalMass));

                        var velA = new Vector3(a.VelocityX, a.VelocityY, a.VelocityZ);
                        var velB = new Vector3(b.VelocityX, b.VelocityY, b.VelocityZ);
                        float relVel = Vector3.Dot(velA - velB, normal);
                        if (relVel < 0)
                        {
                            float impulse = (2 * relVel) / totalMass;
                            velA -= impulse * b.Mass * normal;
                            velB += impulse * a.Mass * normal;
                            a.VelocityX = velA.X; a.VelocityY = velA.Y; a.VelocityZ = velA.Z;
                            b.VelocityX = velB.X; b.VelocityY = velB.Y; b.VelocityZ = velB.Z;
                        }

                        a.LocationX = posA.X; a.LocationY = posA.Y; a.LocationZ = posA.Z;
                        b.LocationX = posB.X; b.LocationY = posB.Y; b.LocationZ = posB.Z;
                        _octree.UpdateParticle(a.OctreeIndex, new Point3D(posA.X, posA.Y, posA.Z));
                        _octree.UpdateParticle(b.OctreeIndex, new Point3D(posB.X, posB.Y, posB.Z));
                    }
                }
            }
        });
    }

    // Method to stop the timer if needed
    public void Stop()
    {
        _timer?.Change(Timeout.Infinite, Timeout.Infinite);
    }
}

