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

ABSL_FLAG(std::string, riskmap_path, "", "Path to the riskmap GeoTIFF file");
ABSL_FLAG(std::string, dem_path, "", "Path to the DEM GeoTIFF file");
ABSL_FLAG(uint16_t, port, 50051, "Port to listen on");

// TODO: Implement calling from gRPC (act as server)

using namespace std;

static void Exit(int code)
{
  cout << "Exit..." << endl;

  GDALDestroy();
  exit(code);
}

int main(int argc, char *argv[])
{
  absl::ParseCommandLine(argc, argv);

  GDALDatasetUniquePtr poDataset;

  GDALAllRegister();

  std::string pszFilenameRiskmap = absl::StrFormat("%s", absl::GetFlag(FLAGS_riskmap_path));
  std::string pszFilenameDEM = absl::StrFormat("%s", absl::GetFlag(FLAGS_dem_path));

  GeoTiffLoader *riskmap_loader = new GeoTiffLoader(pszFilenameRiskmap.c_str());
  cout << "[Riskmap] : " << pszFilenameRiskmap << endl;
  GeoTiffLoader *dem_loader = new GeoTiffLoader(pszFilenameDEM.c_str());
  cout << "[DEM]     : " << pszFilenameDEM << endl;

  cout << "---- [ READY ] ----" << endl;

  auto rq1 = new RouteRequest({2675588.26, 1176002.85}, {2674609.10, 1172793.45}, *riskmap_loader, *dem_loader);
  auto rq2 = new RouteRequest({2674609.10, 1172793.45}, {2677741.71, 1172468.75}, *riskmap_loader, *dem_loader);

  // rq1->run();
  // rq2->run();

  thread t1(&RouteRequest::run, rq1);
  thread t2(&RouteRequest::run, rq2);

  t1.join();
  t2.join();

  riskmap_loader->~GeoTiffLoader();
  dem_loader->~GeoTiffLoader();
  Exit(0);
}