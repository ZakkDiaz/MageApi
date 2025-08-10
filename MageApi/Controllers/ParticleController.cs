using MageUniverse;
using MageUniverse.Physics;
using Microsoft.AspNetCore.Mvc;
using System.Drawing;
using System.Text;

namespace MageApi.Controllers
{
    [ApiController]
    [Route("[controller]")]
    public class ParticleController : Controller
    {
        IUniverse _universe;
        Random _r;
        public ParticleController(IUniverse universe)
        {
            _universe = universe;
            _r = new Random();
        }

        [HttpGet("Clear")]
        public IActionResult Clear()
        {
            _universe.Clear();
            return Ok();
        }

        [HttpPost("AddRandomParticles")]
        public IActionResult AddRandomParticles()
        {
            for(var i = 0; i < 500; i++)
            {
                AddRandomParticle();
            }
            var particle = new Particle
            {
                Mass = 2500,
                LocationX = 250,
                LocationY = 250,
                LocationZ = 0,
                VelocityX = 0,
                VelocityY = 0,
                VelocityZ = 0
            };
            _universe.AddParticle(particle);
            return Ok();
        }

        [HttpPost("AddRandomParticle")]
        public IActionResult AddRandomParticle()
        {
            var particle = GenerateClockwiseParticle(250, 250, .3f, _r);
            _universe.AddParticle(particle);
            return Ok();
        }
        public static Particle GenerateClockwiseParticle(float originX, float originY, float speedScale, Random random)
        {
            // Determine the random radius and angle for the location
            float radius = random.Next(1, 250); // Change 250 to your maximum radius if needed
            double angle = random.NextDouble() * Math.PI * 2;

            // Calculate the particle's location
            float locationX = originX + radius * (float)Math.Cos(angle);
            float locationY = originY + radius * (float)Math.Sin(angle);

            // Calculate the velocity to be perpendicular to the radius vector for clockwise motion
            float velocityX = -radius * (float)Math.Sin(angle) * speedScale;
            float velocityY = radius * (float)Math.Cos(angle) * speedScale;

            return new Particle
            {
                Mass = random.Next(10, 50), // Random mass between 1 and 10
                LocationX = locationX,
                LocationY = locationY,
                LocationZ = 0, // Assuming particles are in a 2D plane
                VelocityX = velocityX,
                VelocityY = velocityY,
                VelocityZ = 0 // Assuming no velocity in the Z direction
            };
        }

        [HttpPost("AddParticle")]
        public IActionResult AddParticle([FromBody] Particle particle)
        {
            _universe.AddParticle(particle);
            return Ok();
        }

        [HttpPost("AddParticles")]
        public IActionResult AddParticles([FromBody] List<Particle> particles)
        {
            _universe.AddParticles(particles);
            return Ok();
        }

        [HttpGet("GetState")]
        public IActionResult GetState()
        {
            return Ok(_universe.GetCurrentState());
        }

        [HttpGet("stream")]
        public async Task StreamSimulationFeed()
        {
            Response.ContentType = "multipart/x-mixed-replace; boundary=frame";

            var clientDisconnectedToken = HttpContext.RequestAborted;
            while (!clientDisconnectedToken.IsCancellationRequested)
            {
                var particles = _universe.GetCurrentState();
                var image = RenderParticlesToBitmap(particles, 500, 500);

                using (var ms = new MemoryStream())
                {
                    // Save the image to the MemoryStream
                    image.Save(ms, System.Drawing.Imaging.ImageFormat.Png); // or PNG, depending on your needs

                    // Convert the image to byte array
                    byte[] imageBytes = ms.ToArray();

                    await Response.Body.WriteAsync(Encoding.ASCII.GetBytes("\r\n--frame\r\n"));
                    await Response.Body.WriteAsync(Encoding.ASCII.GetBytes("Content-Type: image/jpeg\r\n\r\n"));
                    await Response.Body.WriteAsync(imageBytes, 0, imageBytes.Length);
                    await Response.Body.FlushAsync();

                }

                if (clientDisconnectedToken.IsCancellationRequested)
                    break;

                GC.Collect();
            }
        }

        private Bitmap RenderParticlesToBitmap(List<Particle> particles, int width, int height)
        {
            Bitmap bitmap = new Bitmap(width, height);
            using (Graphics g = Graphics.FromImage(bitmap))
            {
                g.Clear(Color.White);
                foreach (var particle in particles)
                {
                    // Simple projection: ignore Z-coordinate
                    float x = particle.LocationX;
                    float y = particle.LocationY;

                    // Draw the particle
                    var size = (int)Math.Max(1, Math.Sqrt(particle.Mass));
                    g.DrawEllipse(Pens.Black, x - (int)size/2, y - (int)size/2, size, size); // drawing small circles for particles
                }
            }
            return bitmap;
        }
    }
}
