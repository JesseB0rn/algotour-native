#include "RouteRequest.h"
#include <cmath>
#include <iostream>
#include <queue>
#include <chrono>
#include <uuid/uuid.h>
#include <mutex>
#include <absl/flags/flag.h>
#include <absl/strings/str_format.h>
#include "helpers.h"

using namespace std;

std::mutex mtx;

RouteRequest::RouteRequest(std::tuple<double, double> start, std::tuple<double, double> end, GeoTiffLoader *riskmap, GeoTiffLoader *dem, std::string basepath) : start(start), end(end), riskmap(riskmap), dem(dem), basepath(basepath)
{
  this->start = start;
  this->end = end;
  this->riskmap = riskmap;
  this->dem = dem;
  this->basepath = basepath;
}

float RouteRequest::walk_time_cost(float demValueA, float demValueB, float distance)
{

  const float hrPerHm = 1.0 / 400.0;
  const float hrPerHmSteep = 150.0; // 1.0 / 100.0;
  const float hrPerSIDist = 1.0 / 4000.0;
  const float cellSize = 10.0;

  const float steepThreshold = tan(40.0 / 180.0 * M_PI);

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
         abs(elevation_diff) * (abs(gradient) >= steepThreshold ? hrPerHmSteep : hrPerHm) * distance;
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

  touched_xmax = 0;
  touched_xmin = riskmap->GetNXSize();
  touched_ymax = 0;
  touched_ymin = riskmap->GetNYSize();

  OGRSpatialReference wgs84;
  OGRSpatialReference epsg2056;

  wgs84.SetWellKnownGeogCS("WGS84");
  epsg2056.importFromEPSG(2056);

  OGRCoordinateTransformation *coordTrans = OGRCreateCoordinateTransformation(&wgs84, &epsg2056);
  coordTrans->Transform(1, &startLat, &startLon);
  coordTrans->Transform(1, &endLat, &endLon);

  coordTrans->DestroyCT(coordTrans);

  // float startLat = 2675588.26;
  // float startLon = 1176002.85;
  // float endLat = 2674609.10;
  // float endLon = 1172793.45;

  int nXSize = riskmap->GetNXSize();
  int nYSize = riskmap->GetNYSize();

  Node end(0, 0);
  Node start(0, 0);
  riskmap->convertLatLonToPixel(startLat, startLon, start.x, start.y);
  riskmap->convertLatLonToPixel(endLat, endLon, end.x, end.y);

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

  if (*riskmap->GetVRefFromDatasetBuffer(startLat, startLon) == NODATA_VALUE || *dem->GetVRefFromDatasetBuffer(startLat, startLon) == NODATA_VALUE)
  {
    cout << "Start is NODATA_VALUE" << endl;
    status = RouteRequestStatus::FAILURE_OUT_OF_BOUNDS;
    return std::vector<Node>();
  }
  if (riskmap->GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE || dem->GetValueFromDatasetBuffer(endLat, endLon) == NODATA_VALUE)
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

  float *demValueEnd = dem->GetVRefFromDatasetBuffer(end.x, end.y);

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
        int i = directionX[current.y * nXSize + current.x];
        int j = directionY[current.y * nXSize + current.x];
        current.x -= i;
        current.y -= j;
      }
      status = RouteRequestStatus::SUCCESS;
      writeSimmilarRaster(accumulatedCost, basepath + "accumulatedCost.tif");
      CPLFree(directionX);
      CPLFree(directionY);
      CPLFree(closedList);
      CPLFree(accumulatedCost);
      return path;
    }

    float *riskmapValue = riskmap->GetVRefFromDatasetBuffer(current.x, current.y);
    float *demValue = dem->GetVRefFromDatasetBuffer(current.x, current.y);

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

        if (closedList[y * nXSize + x])
        {
          continue;
        }

        float *riskmapValueNeighbour = riskmap->GetVRefFromDatasetBuffer(x, y);
        float *demValueNeighbour = dem->GetVRefFromDatasetBuffer(x, y);

        if (*riskmapValueNeighbour == NODATA_VALUE || *demValueNeighbour == NODATA_VALUE)
        {
          continue;
        }

        touched_xmax = max(touched_xmax, x);
        touched_xmin = min(touched_xmin, x);
        touched_ymax = max(touched_ymax, y);
        touched_ymin = min(touched_ymin, y);

        float distance = sqrt(i * i + j * j) * 10.0;
        float cost = walk_time_cost(*demValue, *demValueNeighbour, distance) + 1;

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

        if (neighbour.g >= accumulatedCost[y * nXSize + x])
        {
          continue;
        }

        directionX[y * nXSize + x] = i;
        directionY[y * nXSize + x] = j;
        accumulatedCost[y * nXSize + x] = neighbour.g;
        openList.push(neighbour);
      }
    }
  }
  status = RouteRequestStatus::FAILURE_NO_PATH;

  CPLFree(directionX);
  CPLFree(directionY);
  CPLFree(closedList);
  CPLFree(accumulatedCost);

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
    auto smoothed = Postprocessor::smooth(path, riskmap);

    uuid_t uuid;
    uuid_generate(uuid);
    char uuid_str[37];
    uuid_unparse(uuid, uuid_str);

    filename = string(uuid_str) + ".geojson";

    Postprocessor::writeReprojectedGeoJSON(smoothed, (basepath + filename).c_str());
  }

  auto tend = chrono::high_resolution_clock::now();
  cout << "Found Path, simplify and store in " << chrono::duration_cast<chrono::milliseconds>(tend - tstart).count() << " ms" << endl;

  return RouteRequestStatus::SUCCESS;
}

void RouteRequest::writeSimmilarRaster(float *buffer, std::string path)
{

  int nXSizenew = touched_xmax - touched_xmin;
  int nYSizenew = touched_ymax - touched_ymin;

  cout << "Writing to " << path << endl;

  const char *pszFormat = "GTiff";
  GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
  if (poDriver == nullptr)
  {
    std::cerr << "GTiff driver not available." << std::endl;
    return;
  }

  // set inf and nan to nodata
  for (int i = 0; i < nYSizenew; i++)
  {
    for (int j = 0; j < nXSizenew; j++)
    {

      if (buffer[(i + touched_ymin) * riskmap->GetNXSize() + touched_xmin + j] == INFINITY || isnan(buffer[(i + touched_ymin) * riskmap->GetNXSize() + touched_xmin + j]))
      {
        buffer[(i + touched_ymin) * riskmap->GetNXSize() + touched_xmin + j] = NODATA_VALUE;
      }
    }
  }

  char **papszOptions = nullptr;
  // papszOptions = CSLSetNameValue(papszOptions, "PAM", "YES");
  GDALDataset *poDstDS = poDriver->Create(path.c_str(), nXSizenew, nYSizenew, 1, GDT_Float32, papszOptions);
  // CSLDestroy(papszOptions);
  if (poDstDS == nullptr)
  {
    std::cerr << "Creation of output file failed." << std::endl;
    return;
  }

  GDALRasterBand *poBand = poDstDS->GetRasterBand(1);

  for (int i = 0; i < nYSizenew; i++)
  {
    poBand->RasterIO(GF_Write, 0, i, nXSizenew, 1, buffer + (i + touched_ymin) * riskmap->GetNXSize() + touched_xmin, nXSizenew, 1, GDT_Float32, 0, 0);
  }

  // set the projection and transform
  double adfGeoTransform[6];
  double N_adfGeoTransform[6];
  riskmap->dataset->GetGeoTransform(adfGeoTransform);
  riskmap->dataset->GetGeoTransform(N_adfGeoTransform);
  N_adfGeoTransform[0] = adfGeoTransform[0] + touched_xmin * adfGeoTransform[1];
  N_adfGeoTransform[3] = adfGeoTransform[3] + touched_ymin * adfGeoTransform[5];

  poDstDS->SetGeoTransform(N_adfGeoTransform);
  poDstDS->SetProjection(riskmap->dataset->GetProjectionRef());

  poBand->SetNoDataValue(NODATA_VALUE);

  GDALClose((GDALDatasetH)poDstDS);
}