#include <iostream>
#include <thread>
#include <gdal.h>
#include <gdal_priv.h>
#include "GeoTiffLoader.h"
#include "Postprocessor.h"
#include "QueueItem.h"
#include "RouteRequest.h"

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/strings/str_format.h>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "algotourrouter.grpc.pb.h"

ABSL_FLAG(std::string, riskmap_path, "", "Path to the riskmap GeoTIFF file");
ABSL_FLAG(std::string, dem_path, "", "Path to the DEM GeoTIFF file");
ABSL_FLAG(uint16_t, port, 50051, "Port to listen on");
ABSL_FLAG(std::string, basepath, "~/routes", "Basepath to store routes");

using namespace std;
using grpc::CallbackServerContext;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerUnaryReactor;
using grpc::Status;

using algotourrouter::Algorouter;
using algotourrouter::RouteRequestRPC;
using algotourrouter::RouteResponse;

static void Exit(int code)
{
  cout << "Exit..." << endl;

  GDALDestroy();
  exit(code);
}

void printUsage()
{
  cout << "Usage: ./pathfinder --riskmap_path=path/to/riskmap.tif --dem_path=path/to/dem.tif" << endl;
}

// Logic and data behind the server's behavior.
class ATRouterService final : public Algorouter::CallbackService
{
public:
  ATRouterService(GeoTiffLoader &riskmap_loader, GeoTiffLoader &dem_loader) : riskmap_loader(&riskmap_loader), dem_loader(&dem_loader)
  {
    this->riskmap_loader = &riskmap_loader;
    this->dem_loader = &dem_loader;
  }
  ServerUnaryReactor *DoRouting(CallbackServerContext *context, const RouteRequestRPC *request, RouteResponse *reply) override
  {
    ServerUnaryReactor *reactor = context->DefaultReactor();
    cout << "---- Received request ----" << endl;

    auto rq = new RouteRequest({request->startlat(), request->startlon()}, {request->endlat(), request->endlon()}, *riskmap_loader, *dem_loader, absl::GetFlag(FLAGS_basepath));
    std::string pth;
    RouteRequestStatus _status = (rq->run(pth));

    if (_status != RouteRequestStatus::SUCCESS)
    {
      grpc::Status grpc_status = Status(grpc::StatusCode::INVALID_ARGUMENT, "Invalid request");
      switch (_status)
      {
      case RouteRequestStatus::FAILURE_OUT_OF_BOUNDS:
        grpc_status = Status(grpc::StatusCode::OUT_OF_RANGE, "Out of bounds");
        break;
      case RouteRequestStatus::FAILURE_NO_PATH:
        grpc_status = Status(grpc::StatusCode::NOT_FOUND, "No path found");
        break;
      case RouteRequestStatus::FAILURE_USER_IS_A_DICK:
        grpc_status = Status(grpc::StatusCode::PERMISSION_DENIED, "Start and Endpoint too far apart");
        break;
      case RouteRequestStatus::FAILURE_UNKNOWN:
      case RouteRequestStatus::SUCCESS:
        grpc_status = Status(grpc::StatusCode::INTERNAL, "Internal error");
        break;
      }
      reply->set_pathfile("");
      reactor->Finish(grpc_status);
      return reactor;
    }

    reply->set_pathfile(pth.c_str());

    reactor->Finish(Status::OK);
    cout << "---- Done ----" << endl;
    return reactor;
  };
  GeoTiffLoader *riskmap_loader;
  GeoTiffLoader *dem_loader;
};

void RunServer(uint16_t port, GeoTiffLoader &riskmap_loader, GeoTiffLoader &dem_loader)
{
  cout << "Starting server on port " << port << endl;
  std::string server_address = absl::StrFormat("0.0.0.0:%d", port);

  ATRouterService service(riskmap_loader, dem_loader);

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char *argv[])
{
  absl::ParseCommandLine(argc, argv);
  if (absl::GetFlag(FLAGS_riskmap_path).empty() || absl::GetFlag(FLAGS_dem_path).empty())
  {
    printUsage();
    Exit(1);
  }

  GDALDatasetUniquePtr poDataset;
  GDALAllRegister();

  std::string pszFilenameRiskmap = absl::StrFormat("%s", absl::GetFlag(FLAGS_riskmap_path));
  std::string pszFilenameDEM = absl::StrFormat("%s", absl::GetFlag(FLAGS_dem_path));

  GeoTiffLoader *riskmap_loader = new GeoTiffLoader(pszFilenameRiskmap.c_str());
  GeoTiffLoader *dem_loader = new GeoTiffLoader(pszFilenameDEM.c_str());
  cout << "[Riskmap] : " << pszFilenameRiskmap << endl;
  cout << "[DEM]     : " << pszFilenameDEM << endl;
  // cout << "---- [ READY ] ----" << endl;

  // auto rq1 = new RouteRequest({2675588.26, 1176002.85}, {2674609.10, 1172793.45}, *riskmap_loader, *dem_loader);
  // auto rq2 = new RouteRequest({2674609.10, 1172793.45}, {2677741.71, 1172468.75}, *riskmap_loader, *dem_loader);
  // // rq1->run();
  // // rq2->run();
  // thread t1(&RouteRequest::run, rq1);
  // thread t2(&RouteRequest::run, rq2);
  // t1.join();
  // t2.join();

  RunServer(absl::GetFlag(FLAGS_port), *riskmap_loader, *dem_loader);

  riskmap_loader->~GeoTiffLoader();
  dem_loader->~GeoTiffLoader();
  Exit(0);
}