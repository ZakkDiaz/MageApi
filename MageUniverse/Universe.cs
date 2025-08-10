using System;
using System.Collections.Generic;
using MageUniverse.Physics;
using System.Threading;

namespace MageUniverse
{
    public class Universe : IUniverse
    {
        private List<Particle> _particles = new List<Particle>();
        private IPhysicsEngine _physicsEngine;
        private Timer _timer;
        private const float TargetInterval = 1f / 120f; // Target 30 updates per second
        private DateTime _lastUpdateTime;

        public Universe(IPhysicsEngine physicsEngine)
        {
            _physicsEngine = physicsEngine;
            _lastUpdateTime = DateTime.UtcNow;
            _timer = new Timer(TimerCallback, null, TimeSpan.Zero, TimeSpan.FromSeconds(TargetInterval));
        }

        public void Clear()
        {
            _particles.Clear();
        }

        private void TimerCallback(object state)
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
        }

        public void AddParticles(IEnumerable<Particle> newParticles)
        {
            _particles.AddRange(newParticles);
        }

        public List<Particle> GetCurrentState()
        {
            return new List<Particle>(_particles);
        }

        public void TimeStep(float deltaTime)
        {
            _physicsEngine.TimeStep(_particles, deltaTime);
            foreach(var particle in _particles)
            {
                if(particle.LocationX < 0)
                {
                    particle.LocationX = 0;
                }
                if(particle.LocationY < 0)
                {
                    particle.LocationY = 0;
                }
                if(particle.LocationX > 500)
                {
                    particle.LocationX = 500;
                }
                if(particle.LocationY > 500)
                {
                    particle.LocationY = 500;
                }
            }
        }

        // Method to stop the timer if needed
        public void Stop()
        {
            _timer?.Change(Timeout.Infinite, Timeout.Infinite);
        }
    }
}
