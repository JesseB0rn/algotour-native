#include <iostream>
#include <queue>
#include <thread>
#include <gdal.h>
#include <gdal_priv.h>
#include "GeoTiffLoader.h"

using namespace std;

static void Exit(int code)
{
  cout << "Exit..." << endl;

  GDALDestroy();
  exit(code);
}

typedef struct
{
  int x_riskmap;
  int y_riskmap;
  int x_dem;
  int y_dem;
  float totalCost;
} WalkQueueItem;

typedef struct
{
  int x;
  int y;
} WalkItem;

struct cmp_walkcost
{
  bool operator()(WalkQueueItem left, WalkQueueItem right)
  {
    return left.totalCost > right.totalCost;
  }
};

float walk_time_cost(float demValueA, float demValueB, float distance)
{
  const float cd = 0.25;
  const float ce = 2.5;

  float gradient = (demValueB - demValueA) / distance;
  float elevation_diff = demValueB - demValueA;
  float steepness_cost = abs(gradient) >= tan(45.0 / 180.0 * M_PI) ? 5.0 : 1.0;

  return distance * cd + abs(elevation_diff) * ce * steepness_cost * distance;
}

std::vector<WalkItem> runWalkOnRasters(GeoTiffLoader &riskmap, GeoTiffLoader &dem, int nXSize, int nYSize)
{
  float startLat = 2693252.0;
  float startLon = 1203811.0;

  float endLat = 2692215.73;
  float endLon = 1201934.15;

  WalkItem end = {0, 0};
  riskmap.convertLatLonToPixel(endLat, endLon, end.x, end.y);

  // TODO: Check if start and end are within the bounds of the raster
  // TODO: Check if start and end are not NODATA_VALUE

  auto cmp = [](WalkQueueItem left, WalkQueueItem right)
  { return left.totalCost > right.totalCost; };
  priority_queue<WalkQueueItem, std::vector<WalkQueueItem>, cmp_walkcost> q = std::priority_queue<WalkQueueItem, std::vector<WalkQueueItem>, cmp_walkcost>();

  WalkQueueItem start = {0, 0, 0, 0, 0};
  riskmap.convertLatLonToPixel(startLat, startLon, start.x_riskmap, start.y_riskmap);
  dem.convertLatLonToPixel(startLat, startLon, start.x_dem, start.y_dem);
  q.push(start);

  auto cameFrom = std::unordered_map<int, WalkItem>();

  bool *visited = (bool *)CPLMalloc(sizeof(bool) * nXSize * nYSize);

  while (!q.empty())
  {
    WalkQueueItem current = q.top();
    q.pop();

    if (visited[current.x_riskmap + current.y_riskmap * nXSize])
    {
      continue;
    }
    visited[current.x_riskmap + current.y_riskmap * nXSize] = true;

    float *riskmapValue = riskmap.GetVRefFromDatasetBuffer(current.x_riskmap, current.y_riskmap);
    float *demValue = dem.GetVRefFromDatasetBuffer(current.x_dem, current.y_dem);

    // cout << "Riskmap value: " << *riskmapValue << " DEM value: " << *demValue << endl;

    if (current.x_riskmap == end.x && current.y_riskmap == end.y)
    {
      auto path = std::vector<WalkItem>();
      WalkItem current = {end.x, end.y};
      while (current.x != start.x_riskmap || current.y != start.y_riskmap)
      {
        path.push_back(current);
        current = cameFrom[current.x + current.y * nXSize];
      }
      return path;
    }

    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        WalkQueueItem next = {current.x_riskmap + i, current.y_riskmap + j, current.x_dem + i, current.y_dem + j, current.totalCost};

        if (next.x_riskmap < 0 || next.x_riskmap >= nXSize || next.y_riskmap < 0 || next.y_riskmap >= nYSize || i == 0 && j == 0)
        {
          continue;
        }

        if (next.x_dem < 0 || next.x_dem >= nXSize || next.y_dem < 0 || next.y_dem >= nYSize)
        {
          continue;
        }

        float *nextRiskmapValue = riskmap.GetVRefFromDatasetBuffer(next.x_riskmap, next.y_riskmap);
        float *nextDemValue = dem.GetVRefFromDatasetBuffer(next.x_dem, next.y_dem);

        if (*nextRiskmapValue == NODATA_VALUE || *nextDemValue == NODATA_VALUE)
        {
          continue;
        }

        float heuristic = sqrt((end.x - next.x_riskmap) * (end.x - next.x_riskmap) + (end.y - next.y_riskmap) * (end.y - next.y_riskmap));
        next.totalCost += 50.0 * (*nextRiskmapValue) + walk_time_cost(*demValue, *nextDemValue, sqrt(i * i + j * j) * 10.0) + heuristic * 0.1;

        cameFrom[next.x_riskmap + next.y_riskmap * riskmap.GetNXSize()] = {current.x_riskmap, current.y_riskmap};
        q.push(next);
      }
    }
  }

  cout << "Path found" << endl;
}

int main(int argc, char *argv[])
{

  GDALDatasetUniquePtr poDataset;
  GDALAllRegister();

  argc = GDALGeneralCmdLineProcessor(argc, &argv, 0);

  const char *pszFilenameRiskmap = argv[1];
  const char *pszFilenameDEM = argv[2];

  GeoTiffLoader *riskmap_loader = new GeoTiffLoader(pszFilenameRiskmap);
  cout << "Riskmap" << pszFilenameRiskmap << "read" << endl;
  GeoTiffLoader *dem_loader = new GeoTiffLoader(pszFilenameDEM);
  cout << "DEM" << pszFilenameDEM << "read" << endl;

  cout << "Rasters read successfully, ready" << endl;

  float a = riskmap_loader->GetValueFromDatasetBuffer(2662390.000000, 1239740.000000);
  float b = dem_loader->GetValueFromDatasetBuffer(2662390.000000, 1239740.000000);

  cout << "Value at 2662390.000000, 1239740.000000: " << a << "@ " << b << "müM" << endl;

  for (int i = 0; i < 10; i++)
  {
    runWalkOnRasters(*riskmap_loader, *dem_loader, riskmap_loader->GetNXSize(), riskmap_loader->GetNYSize());
  }

  riskmap_loader->~GeoTiffLoader();
  dem_loader->~GeoTiffLoader();
  Exit(0);
}