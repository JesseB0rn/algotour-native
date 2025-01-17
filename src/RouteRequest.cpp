#include "RouteRequest.h"
#include <cmath>
#include <iostream>
#include <queue>
#include <chrono>
#include <uuid/uuid.h>
#include <mutex>
#include "helpers.h"

using namespace std;

std::mutex mtx;

RouteRequest::RouteRequest(std::tuple<double, double> start, std::tuple<double, double> end, GeoTiffLoader &riskmap, GeoTiffLoader &dem) : start(start), end(end), riskmap(riskmap), dem(dem)
{
  this->start = start;
  this->end = end;
  this->riskmap = riskmap;
  this->dem = dem;
}

float RouteRequest::walk_time_cost(float demValueA, float demValueB, float distance)
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

std::vector<Node> RouteRequest::runWalkOnRasters(RouteRequestStatus &status, double *progress)
{
  // float startLat = 269 325 2.0;
  // float startLon = 1203811.0;
  // float endLat = 2692215.73;
  // float endLon = 1201934.15;

  // 2'675'588.26, 1'176'002.85
  // 2'674'609.10, 1'172'793.45

  double startLat, startLon, endLat, endLon;
  std::tie(startLat, startLon) = start;
  std::tie(endLat, endLon) = end;

  OGRSpatialReference wgs84;
  OGRSpatialReference epsg2056;

  wgs84.SetWellKnownGeogCS("WGS84");
  epsg2056.importFromEPSG(2056);

  OGRCoordinateTransformation *coordTrans = OGRCreateCoordinateTransformation(&wgs84, &epsg2056);
  coordTrans->Transform(1, &startLat, &startLon);
  coordTrans->Transform(1, &endLat, &endLon);

  // float startLat = 2675588.26;
  // float startLon = 1176002.85;
  // float endLat = 2674609.10;
  // float endLon = 1172793.45;

  int nXSize = riskmap.GetNXSize();
  int nYSize = riskmap.GetNYSize();

  Node end(0, 0);
  Node start(0, 0);
  riskmap.convertLatLonToPixel(startLat, startLon, start.x, start.y);
  riskmap.convertLatLonToPixel(endLat, endLon, end.x, end.y);

  if (start.x <= 0 || start.x > nXSize || start.y <= 0 || start.y > nYSize)
  {
    cout << "Start is out of bounds" << endl;
    status = RouteRequestStatus::FAILURE_OUT_OF_BOUNDS;
    return std::vector<Node>();
  }
  if (end.x <= 0 || end.x > nXSize || end.y <= 0 || end.y > nYSize)
  {
    cout << "End is out of bounds" << endl;
    status = RouteRequestStatus::FAILURE_OUT_OF_BOUNDS;
    return std::vector<Node>();
  }

  if (*riskmap.GetVRefFromDatasetBuffer(startLat, startLon) == NODATA_VALUE || *dem.GetVRefFromDatasetBuffer(startLat, startLon) == NODATA_VALUE)
  {
    cout << "Start is NODATA_VALUE" << endl;
    status = RouteRequestStatus::FAILURE_OUT_OF_BOUNDS;
    return std::vector<Node>();
  }
  if (riskmap.GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE || dem.GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE)
  {
    cout << "End is NODATA_VALUE" << endl;
    status = RouteRequestStatus::FAILURE_OUT_OF_BOUNDS;
    return std::vector<Node>();
  }

  if ((startLat - endLat) * (startLat - endLat) + (startLon - endLon) * (startLon - endLon) >= 20000 * 20000)
  {
    cout << "Start and end are too far apart" << endl;
    status = RouteRequestStatus::FAILURE_USER_IS_A_DICK;

    return std::vector<Node>();
  };

  float totalDistance = sqrt((start.x - end.x) * (start.x - end.x) + (start.y - end.y) * (start.y - end.y));

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
      status = RouteRequestStatus::SUCCESS;
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

        float remDistance = sqrt((end.x - x) * (end.x - x) + (end.y - y) * (end.y - y));

        if (progress)
        {
          *progress = remDistance / totalDistance;
        }

        Node neighbour(x, y);
        neighbour.g = current.g + (*riskmapValueNeighbour * 50.0) + cost;

        neighbour.h = walk_time_cost(*demValue, *demValue, remDistance);
        // neighbour.h = 0.0;
        neighbour.f = neighbour.g + neighbour.h;

        if (Helpers::distanceToLineBoundsCheck(start, current, end) > 1000.00)
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
  status = RouteRequestStatus::FAILURE_NO_PATH;
  return std::vector<Node>();
}

RouteRequestStatus RouteRequest::run(std::string &filename, double *progress)
{
  auto tstart = chrono::high_resolution_clock::now();

  RouteRequestStatus status = RouteRequestStatus::FAILURE_UNKNOWN;
  std::vector<Node> path = runWalkOnRasters(status, progress);

  if (status != RouteRequestStatus::SUCCESS)
  {
    cout << "Failed to find path" << endl;
    return status;
  }
  // string filename;
  {
    std::lock_guard<std::mutex> lock(mtx);

    Postprocessor::simplify(path);
    auto smoothed = Postprocessor::smooth(path, &riskmap);

    uuid_t uuid;
    uuid_generate(uuid);
    char uuid_str[37];
    uuid_unparse(uuid, uuid_str);

    const char *basepath = "/Users/jesseb0rn/Documents/repos/algotour-native/out/";
    filename = basepath + string(uuid_str) + ".geojson";

    Postprocessor::writeReprojectedGeoJSON(smoothed, filename.c_str());
  }

  auto tend = chrono::high_resolution_clock::now();
  cout << "Found Path, simplify and store in " << chrono::duration_cast<chrono::milliseconds>(tend - tstart).count() << " ms" << endl;

  return RouteRequestStatus::SUCCESS;
}