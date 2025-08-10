using MageUniverse.Physics;
using Microsoft.Extensions.DependencyInjection;

namespace MageUniverse
{
    public static class ServiceCollectionExtensions
    {
        public static IServiceCollection AddUniversePhysics(this IServiceCollection services)
        {
            services.AddSingleton<IPhysicsEngine, PhysicsEngine>();
            services.AddSingleton<IUniverse, Universe>();

            return services;
        }
    }
}
