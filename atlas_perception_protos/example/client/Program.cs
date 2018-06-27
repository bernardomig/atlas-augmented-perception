using System;
using System.Threading;
using System.Threading.Tasks;
using System.Net;
using System.Net.Http;

using Atlas.Augmented;

namespace Client
{
    static class Program
    {
        static HttpClient client = new HttpClient();
        static public async Task Main(string[] args)
        {
            client.BaseAddress = new Uri("http://192.168.0.107:5001");

            var detectedObjects = await GetObject();

            var obj1 = detectedObjects.DetectedObjects_[0];

            Console.WriteLine(obj1.BoundingBox.Size);
        }

        static async Task<DetectedObjects> GetObject()
        {
            var resp = await client.GetAsync("/");

            if(resp.IsSuccessStatusCode) {
                var body = resp.Content.ReadAsByteArrayAsync();
                var detectedObject = DetectedObjects.Parser.ParseFrom(await body);
                return detectedObject;
            }
            else
            {
                return null;
            }
        }

    }
}
