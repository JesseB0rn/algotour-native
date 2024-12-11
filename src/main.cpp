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

  const float hrPerHm = 1.0 / 400.0;
  const float hrPerHmSteep = 1.0 / 100.0;
  const float hrPerSIDist = 1.0 / 4000.0;
  const float cellSize = 10.0;

  const float steepThreshold = tan(45.0 / 180.0 * M_PI);

  float elevation_diff = demValueB - demValueA;
  float gradient = elevation_diff / distance;

  /**
   * h(x) needs to be at most:
   * 0.2 for height diff
   * 0.0035 for distance
   * For riskcost: 50.0
   *
   * ----
   * at least:
   *
   * 0.0025 for distance
   * 0.0 for height diff
   * 0.0 for riskcost
   */

  return distance * hrPerSIDist +
         abs(elevation_diff) * (abs(gradient) >= steepThreshold ? hrPerHmSteep : hrPerHm);
}

std::vector<Node> runWalkOnRasters(GeoTiffLoader &riskmap, GeoTiffLoader &dem, int nXSize, int nYSize)
{
  // float startLat = 269 325 2.0;
  // float startLon = 1203811.0;
  // float endLat = 2692215.73;
  // float endLon = 1201934.15;

  // 2'675'588.26, 1'176'002.85
  // 2'674'609.10, 1'172'793.45

  float startLat = 2675588.26;
  float startLon = 1176002.85;
  float endLat = 2674609.10;
  float endLon = 1172793.45;

  Node end(0, 0);
  Node start(0, 0);
  riskmap.convertLatLonToPixel(startLat, startLon, start.x, start.y);
  riskmap.convertLatLonToPixel(endLat, endLon, end.x, end.y);

  if (*riskmap.GetVRefFromDatasetBuffer(startLat, startLon) == NODATA_VALUE || *dem.GetVRefFromDatasetBuffer(startLat, startLon) == NODATA_VALUE)
  {
    cout << "Start is NODATA_VALUE" << endl;
    return std::vector<Node>();
  }
  if (riskmap.GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE || dem.GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE)
  {
    cout << "End is NODATA_VALUE" << endl;
    return std::vector<Node>();
  }

  if ((startLat - endLat) * (startLat - endLat) + (startLon - endLon) * (startLon - endLon) >= 20000 * 20000)
  {
    cout << "Start and end are too far apart" << endl;
    return std::vector<Node>();
  };

  float *demValueEnd = dem.GetVRefFromDatasetBuffer(end.x, end.y);

  priority_queue<Node, std::vector<Node>, greater<Node>> openList;
  openList.push(start);

  int8_t *directionX = (int8_t *)CPLMalloc(sizeof(int8_t) * nXSize * nYSize);
  int8_t *directionY = (int8_t *)CPLMalloc(sizeof(int8_t) * nXSize * nYSize);
  bool *closedList = (bool *)CPLMalloc(sizeof(bool) * nXSize * nYSize);
  float *accumulatedCost = (float *)CPLMalloc(sizeof(float) * nXSize * nYSize);

  std::fill(closedList, closedList + nXSize * nYSize, false);
  std::fill(directionX, directionX + nXSize * nYSize, 255);
  std::fill(directionY, directionY + nXSize * nYSize, 255);
  std::fill(accumulatedCost, accumulatedCost + nXSize * nYSize, INFINITY);

  cout << "Start: " << start.x << ", " << start.y << endl;

  while (!openList.empty())
  {
    // cout << "Openlist size: " << openList.size() << endl;
    Node current = openList.top();
    openList.pop();

    if (current == end)
    {
      cout << "Path found, backtracking..." << endl;
      std::vector<Node> path = std::vector<Node>();
      while (!(current == start))
      {
        // cout << "Path item: " << current.x << ", " << current.y << endl;
        path.push_back(current);
        int i = directionX[current.x * nYSize + current.y];
        int j = directionY[current.x * nYSize + current.y];
        current.x -= i;
        current.y -= j;
      }
      return path;
    }

    float *riskmapValue = riskmap.GetVRefFromDatasetBuffer(current.x, current.y);
    float *demValue = dem.GetVRefFromDatasetBuffer(current.x, current.y);

    closedList[current.x * nYSize + current.y] = true;
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        if (i == 0 && j == 0)
        {
          continue;
        }

        int x = current.x + i;
        int y = current.y + j;

        if (x <= 0 || x > nXSize || y <= 0 || y > nYSize)
        {
          continue;
        }

        if (closedList[x * nYSize + y])
        {
          continue;
        }

        float *riskmapValueNeighbour = riskmap.GetVRefFromDatasetBuffer(x, y);
        float *demValueNeighbour = dem.GetVRefFromDatasetBuffer(x, y);

        if (*riskmapValueNeighbour == NODATA_VALUE || *demValueNeighbour == NODATA_VALUE)
        {
          continue;
        }

        float distance = sqrt(i * i + j * j) * 10.0;
        float cost = walk_time_cost(*demValue, *demValueNeighbour, distance);

        // cout << "Cost: " << cost << endl;

        Node neighbour(x, y);
        neighbour.g = current.g + (*riskmapValueNeighbour * 50.0) + cost;
        neighbour.h = walk_time_cost(*demValue, *demValue, sqrt((end.x - x) * (end.x - x) + (end.y - y) * (end.y - y)));
        // neighbour.h = 0.0;
        neighbour.f = neighbour.g + neighbour.h;

        if (neighbour.h > 6000)
        {
          continue;
        }

        if (neighbour.g >= accumulatedCost[x * nYSize + y])
        {
          continue;
        }

        directionX[x * nYSize + y] = i;
        directionY[x * nYSize + y] = j;
        accumulatedCost[x * nYSize + y] = neighbour.g;
        openList.push(neighbour);
      }
    }
  }

  return std::vector<Node>();
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
  // 2676023.8,1175322.7
  // for (int i = 0; i < 10; i++)
  // {
  //   int x, y;
  //   cin >> x;
  //   cin >> y;
  //   float a = riskmap_loader->GetValueFromDatasetBuffer(x, y);
  //   float b = dem_loader->GetValueFromDatasetBuffer(x, y);
  //   cout << "Value at " << x << ", " << y << ": " << a << "@ " << b << "müM" << endl;
  // }
  // Exit(0);

  auto pth = runWalkOnRasters(*riskmap_loader, *dem_loader, riskmap_loader->GetNXSize(), riskmap_loader->GetNYSize());
  // for (auto &item : pth)
  // {
  //   double lat, lon;
  //   riskmap_loader->convertPixelToLatLon(item.x, item.y, lat, lon);
  //   cout << "Path item: " << lat << ", " << lon << endl;
  // }
  Postprocessor *po = new Postprocessor(pth);
  // po->simplify();
  po->writeReprojectedGeoJSON("/Users/jesseb0rn/Documents/repos/algotour-native/out.json", riskmap_loader);

  riskmap_loader->~GeoTiffLoader();
  dem_loader->~GeoTiffLoader();
  Exit(0);
}