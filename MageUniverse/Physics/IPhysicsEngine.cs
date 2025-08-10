namespace MageUniverse.Physics
{
    public interface IPhysicsEngine
    {
        void TimeStep(List<Particle> particles, float deltaTime);
    }
}