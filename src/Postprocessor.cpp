#include <iostream>
#include "Postprocessor.h"
#include "QueueItem.h"
#include <gdal_priv.h>
#include <ogrsf_frmts.h>

using namespace std;

// Postprocessor::~Postprocessor() {}

void Postprocessor::writeReprojectedGeoJSON(const std::vector<std::tuple<double, double>> &path, const char *filename)
{
  GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName("GeoJSON");
  if (!poDriver)
  {
    std::cerr << "Failed to get GeoJSON driver" << std::endl;
    return;
  }

  GDALDataset *hDstDS = poDriver->Create(filename, 0, 0, 0, GDT_Unknown, NULL);
  if (!hDstDS)
  {
    std::cerr << "Failed to create GeoJSON file" << std::endl;
    return;
  }

  OGRSpatialReference *oSRS = new OGRSpatialReference();
  oSRS->importFromEPSG(2056);

  // set RFC7946=YES to get on the fly reprojection to WGS84
  char **papszOptions = NULL;
  papszOptions = CSLSetNameValue(papszOptions, "RFC7946", "YES");

  OGRLayer *hLayer = hDstDS->CreateLayer("path", oSRS, wkbLineString, papszOptions);
  if (!hLayer)
  {
    std::cerr << "Failed to create layer" << std::endl;
    return;
  }

  OGRLineString *line = new OGRLineString();
  for (auto p : path)
  {
    double lat, lon;
    std::tie(lat, lon) = p;
    line->addPoint(lat, lon);
  }

  OGRFeature *poFeature = OGRFeature::CreateFeature(hLayer->GetLayerDefn());
  poFeature->SetGeometry(line);
  if (OGR_L_CreateFeature(hLayer, poFeature) != OGRERR_NONE)
  {
    std::cerr << "Failed to create feature" << std::endl;
    return;
  }

  OGRGeometryFactory::destroyGeometry(line);
  CSLDestroy(papszOptions);
  delete oSRS;
  OGRFeature::DestroyFeature(poFeature);

  GDALClose(hDstDS);
}

/**
 * @brief Simplify the path using the Douglas-Peucker algorithm
 * function DouglasPeucker(PointList[], epsilon)
    // Finde den Punkt mit dem größten Abstand
    dmax = 0
    index = 0
    for i = 2 to (length(PointList) −1)
        d = LotrechterAbstand(PointList[i], Line(PointList[1], PointList[end]))
        if d > dmax
            index = i
            dmax = d
    // Wenn die maximale Entfernung größer als Epsilon ist, dann rekursiv vereinfachen
    if dmax > epsilon
        // Recursive call
        recResults1[] = DouglasPeucker(PointList[1...index], epsilon)
        recResults2[] = DouglasPeucker(PointList[index...end], epsilon)
        // Ergebnisliste aufbauen
        ResultList[] = {recResults1[1...end-1], recResults2[1...end]}
    else
        ResultList[] = {PointList[1], PointList[end]}
    // Ergebnis zurückgeben
    return ResultList[]
end
 *
 *
 */
void Postprocessor::simplify(std::vector<Node> &items)
{
  int oldSize = items.size();

  std::vector<Node> newItems = std::vector<Node>();
  items = Postprocessor::douglasPeucker(items.begin(), items.end() - 1, 5.5);

  int newSize = items.size();
  std::cout << "Simplified path from " << oldSize << " to " << newSize << " points" << std::endl;
}

double Postprocessor::distanceToLine(Node pA, Node pTest, Node pB)
{
  double A = pB.y - pA.y;
  double B = pA.x - pB.x;
  double C = pB.x * pA.y - pA.x * pB.y;

  return abs(A * pTest.x + B * pTest.y + C) / sqrt(A * A + B * B);
}
std::vector<Node> Postprocessor::douglasPeucker(std::vector<Node>::iterator start, std::vector<Node>::iterator end, double epsilon)
{
  std::vector<Node> resultsList = {};

  auto maxDistance = 0.0;
  auto farthestIndex = start;

  for (auto it = start + 1; it != end; ++it)
  {
    double distance = distanceToLine(*start, *it, *end);
    if (distance > maxDistance)
    {
      maxDistance = distance;
      farthestIndex = it;
    }
  }

  if (maxDistance > epsilon)
  {

    auto recResults1 = douglasPeucker(start, farthestIndex, epsilon);
    auto recResults2 = douglasPeucker(farthestIndex, end, epsilon);
    resultsList.insert(resultsList.end(), recResults1.begin(), recResults1.end() - 1);
    resultsList.insert(resultsList.end(), recResults2.begin(), recResults2.end());
  }
  else
  {
    resultsList = {*start, *end};
  }
  return resultsList;
}

/**
 * @brief Smooth using Chaikin's algorithm
 */
std::vector<std::tuple<double, double>> Postprocessor::smooth(std::vector<Node> &items, GeoTiffLoader *riskmap)
{
  int oldSize = items.size();
  std::vector<std::tuple<double, double>> newItems = std::vector<std::tuple<double, double>>();

  // convert Node(screen coordinates) to tuple (geographical coordinates)
  for (auto p : items)
  {
    double lat, lon;
    riskmap->convertPixelToLatLon(p.x, p.y, lat, lon);
    newItems.push_back(std::make_tuple(lat, lon));
  }

  // Chaikin's algorithm for 3 iterations
  for (int iters = 0; iters < 3; iters++)
  {
    std::vector<std::tuple<double, double>> smoothed = std::vector<std::tuple<double, double>>();
    for (int i = 0; i < newItems.size() - 1; i++)
    {
      double x1, y1, x2, y2;
      std::tie(x1, y1) = newItems[i];
      std::tie(x2, y2) = newItems[i + 1];

      double x1new = 0.75 * x1 + 0.25 * x2;
      double y1new = 0.75 * y1 + 0.25 * y2;
      double x2new = 0.25 * x1 + 0.75 * x2;
      double y2new = 0.25 * y1 + 0.75 * y2;

      smoothed.push_back(std::make_tuple(x1new, y1new));
      smoothed.push_back(std::make_tuple(x2new, y2new));
    }
    newItems = smoothed;
  }

  return newItems;
}