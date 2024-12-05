#include <iostream>
#include <gdal.h>
#include <gdal_priv.h>
#include "GeoTiffLoader.h"

using namespace std;

template <typename T>
struct Tuple
{
  T a;
  T b;

  Tuple(T f, T s) : a(f), b(s) {}
};

// static Tuple<double> GetWorldCoordinates(int x, int y, double *fwdTransform)
// {

//   double worldX = fwdTransform[0] + x * fwdTransform[1];
//   double worldY = fwdTransform[3] + y * fwdTransform[5];

//   return Tuple<double>(worldX, worldY);
// }

// static Tuple<int> GetPixelCoordinates(double x, double y, double *fwdTransform)
// {
//   int pixelY = (y - fwdTransform[0]) / fwdTransform[1];
//   int pixelX = (x - fwdTransform[3]) / fwdTransform[5];

//   return Tuple<int>(pixelX, pixelY);
// }

static void Exit(int code)
{
  cout << "Exiting..." << endl;

  GDALDestroy();

  exit(code);
}

int main(int argc, char *argv[])
{

  GDALDatasetUniquePtr poDataset;
  GDALAllRegister();

  argc = GDALGeneralCmdLineProcessor(argc, &argv, 0);

  const char *pszFilenameRiskmap = argv[1];
  const char *pszFilenameDEM = argv[2];

  auto riskmap_loader = new GeoTiffLoader(pszFilenameRiskmap);
  cout << "Riskmap" << pszFilenameRiskmap << "read" << endl;
  auto dem_loader = new GeoTiffLoader(pszFilenameDEM);
  cout << "DEM" << pszFilenameDEM << "read" << endl;

  cout << "Rasters read successfully, ready" << endl;

  float a = riskmap_loader->GetValueFromDatasetBuffer(2662390.000000, 1239740.000000);
  float b = dem_loader->GetValueFromDatasetBuffer(2662390.000000, 1239740.000000);

  cout << "Value at 2662390.000000, 1239740.000000: " << a << "@ " << b << "müM" << endl;

  riskmap_loader->~GeoTiffLoader();
  dem_loader->~GeoTiffLoader();
  Exit(0);
}