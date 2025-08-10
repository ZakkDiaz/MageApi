using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using ParticleLib.Modern.Models._3D;

namespace MageUniverse;

public static class ServiceCollectionExtensions
{
    public static IServiceCollection AddUniverse(this IServiceCollection services, IConfiguration configuration)
    {
        var boundsSection = configuration.GetSection("Universe:Bounds");
        var min = new Point3D(
            boundsSection.GetValue<float>("MinX", 0),
            boundsSection.GetValue<float>("MinY", 0),
            boundsSection.GetValue<float>("MinZ", 0));
        var max = new Point3D(
            boundsSection.GetValue<float>("MaxX", 500),
            boundsSection.GetValue<float>("MaxY", 500),
            boundsSection.GetValue<float>("MaxZ", 500));

        services.AddSingleton<IUniverse>(_ => new Universe(new AAABBB(min, max)));

        return services;
    }
}

