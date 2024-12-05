#include <iostream>
#include <queue>
#include <thread>
#include <gdal.h>
#include <gdal_priv.h>
#include <uuid/uuid.h>
#include "GeoTiffLoader.h"
#include "Postprocessor.h"
#include "QueueItem.h"

using namespace std;

static void Exit(int code)
{
  cout << "Exit..." << endl;

  GDALDestroy();
  exit(code);
}

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

  if (riskmap.GetValueFromDatasetBuffer(startLat, startLon) == NODATA_VALUE || dem.GetValueFromDatasetBuffer(startLat, startLon) == NODATA_VALUE)
  {
    cout << "Start is NODATA_VALUE" << endl;
    return std::vector<WalkItem>();
  }
  if (riskmap.GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE || dem.GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE)
  {
    cout << "End is NODATA_VALUE" << endl;
    return std::vector<WalkItem>();
  }

  if ((startLat - endLat) * (startLat - endLat) + (startLon - endLon) * (startLon - endLon) >= 20000 * 20000)
  {
    cout << "Start and end are too far apart" << endl;
    return std::vector<WalkItem>();
  };

  auto cmp = [](WalkQueueItem left, WalkQueueItem right)
  { return left.totalCost > right.totalCost; };
  priority_queue<WalkQueueItem, std::vector<WalkQueueItem>, cmp_walkcost> q = std::priority_queue<WalkQueueItem, std::vector<WalkQueueItem>, cmp_walkcost>();

  WalkQueueItem start = {0, 0, 0, 0, 0};
  riskmap.convertLatLonToPixel(startLat, startLon, start.x_riskmap, start.y_riskmap);
  dem.convertLatLonToPixel(startLat, startLon, start.x_dem, start.y_dem);
  q.push(start);

  float maxHeuristic = max(abs(end.x - start.x_riskmap), abs(end.y - start.y_riskmap));

  auto cameFrom = std::unordered_map<int, WalkItem>();

  bool *visited = (bool *)CPLMalloc(sizeof(bool) * nXSize * nYSize);
  std::fill(visited, visited + nXSize * nYSize, false);

  float *accumulatedCost = (float *)CPLMalloc(sizeof(float) * nXSize * nYSize);
  std::fill(accumulatedCost, accumulatedCost + nXSize * nYSize, INFINITY);

  while (!q.empty())
  {
    WalkQueueItem current = q.top();
    q.pop();

    if (accumulatedCost[current.x_riskmap + current.y_riskmap * nXSize] < current.totalCost)
    {
      continue;
    }
    accumulatedCost[current.x_riskmap + current.y_riskmap * nXSize] = current.totalCost;

    float *riskmapValue = riskmap.GetVRefFromDatasetBuffer(current.x_riskmap, current.y_riskmap);
    float *demValue = dem.GetVRefFromDatasetBuffer(current.x_dem, current.y_dem);

    // cout << "Riskmap value: " << *riskmapValue << " DEM value: " << *demValue << endl;

    if (current.x_riskmap == end.x && current.y_riskmap == end.y)
    {
      auto path = std::vector<WalkItem>();
      cout << "Path found" << endl;
      WalkItem curr = {end.x, end.y};

      while ((curr.x != start.x_riskmap || curr.y != start.y_riskmap))
      {
        path.push_back(curr);
        curr = cameFrom[curr.x + curr.y * nXSize];
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

        float heuristic = max(abs(end.x - next.x_riskmap), abs(end.y - next.y_riskmap));
        if (heuristic > 2 * maxHeuristic)
        {
          continue;
        }
        /**
         * Heuristic calculation:
         * -  cost for riskmap:         0     | 50.0
         * -  cost for flat wt:
         *
         */
        next.totalCost += 50.0 * (*nextRiskmapValue) + walk_time_cost(*demValue, *nextDemValue, sqrt(i * i + j * j) * 10.0) + heuristic;

        if (accumulatedCost[next.x_riskmap + next.y_riskmap * nXSize] < next.totalCost)
        {
          continue;
        }
        cameFrom[next.x_riskmap + next.y_riskmap * riskmap.GetNXSize()] = {current.x_riskmap, current.y_riskmap};
        q.push(next);
      }
    }
  }

  cout << "Path found" << endl;
  return std::vector<WalkItem>();
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

  // float a = riskmap_loader->GetValueFromDatasetBuffer(2662390.000000, 1239740.000000);
  // float b = dem_loader->GetValueFromDatasetBuffer(2662390.000000, 1239740.000000);

  // cout << "Value at 2662390.000000, 1239740.000000: " << a << "@ " << b << "müM" << endl;

  auto pth = runWalkOnRasters(*riskmap_loader, *dem_loader, riskmap_loader->GetNXSize(), riskmap_loader->GetNYSize());
  // for (auto &item : pth)
  // {
  //   double lat, lon;
  //   riskmap_loader->convertPixelToLatLon(item.x, item.y, lat, lon);
  //   cout << "Path item: " << lat << ", " << lon << endl;
  // }
  Postprocessor *po = new Postprocessor(pth);
  po->simplify();
  po->writeReprojectedGeoJSON("/Users/jesseb0rn/Documents/repos/algotour-native/out.json", riskmap_loader);

  riskmap_loader->~GeoTiffLoader();
  dem_loader->~GeoTiffLoader();
  Exit(0);
}