using System;
using System.Collections.Generic;
using System.Threading;
using MageUniverse.Physics;
using ParticleLib.Modern.Models._3D;

namespace MageUniverse;

public class Universe : IUniverse
{
    private readonly List<Particle> _particles = new();
    private readonly IPhysicsEngine _physicsEngine;
    private readonly Timer _timer;
    private readonly Octree _octree;
    private const float TargetInterval = 1f / 120f; // Target 120 updates per second
    private DateTime _lastUpdateTime;

    public Universe(IPhysicsEngine physicsEngine)
    {
        _physicsEngine = physicsEngine;
        _octree = new Octree(new AAABBB(new Point3D(0, 0, 0), new Point3D(500, 500, 500)));
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
        _physicsEngine.TimeStep(_particles, _octree, deltaTime);

        foreach (var particle in _particles)
        {
            if (particle.LocationX < 0) particle.LocationX = 0;
            if (particle.LocationY < 0) particle.LocationY = 0;
            if (particle.LocationX > 500) particle.LocationX = 500;
            if (particle.LocationY > 500) particle.LocationY = 500;

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

