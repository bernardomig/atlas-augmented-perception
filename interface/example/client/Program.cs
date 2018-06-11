using System;
using System.Threading;
using System.Threading.Tasks;

using Grpc.Core;
using Atlas.Augmented;

namespace Client
{
    class Program
    {
        static async Task GetDetectedObjects(DetectionService.DetectionServiceClient client) {

            try {
                var call = client.GetDetectedObjects(new DetectionServiceRequest());

                var responseStream = call.ResponseStream;

                while(await responseStream.MoveNext(CancellationToken.None)) {
                    Console.WriteLine("Received message");
                    Console.WriteLine(responseStream.Current);
                }
            }
            catch (RpcException) {
                throw;
            }
        }

        static void Main(string[] args)
        {
            Channel channel = new Channel("[::1]:5000", ChannelCredentials.Insecure);

            var client = new DetectionService.DetectionServiceClient(channel);

            GetDetectedObjects(client).Wait();

            channel.ShutdownAsync().Wait();
            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
        }
    }
}
