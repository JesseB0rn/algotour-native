#include <iostream>
#include "Postprocessor.h"
#include "QueueItem.h"
#include <gdal_priv.h>
#include <ogrsf_frmts.h>

using namespace std;

Postprocessor::Postprocessor(const std::vector<Node> &items) : items(items)
{
  this->items = items;
}

// Postprocessor::~Postprocessor() {}

void Postprocessor::writeReprojectedGeoJSON(const char *filename, GeoTiffLoader *riskmap)
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
  for (auto item : items)
  {
    double lat, lon;
    riskmap->convertPixelToLatLon(item.x, item.y, lat, lon);
    // cout << "Path item: " << lat << ", " << lon << endl;
    line->addPoint(lat, lon);
  }

  OGRFeature *poFeature = OGRFeature::CreateFeature(hLayer->GetLayerDefn());
  poFeature->SetGeometry(line);
  if (OGR_L_CreateFeature(hLayer, poFeature) != OGRERR_NONE)
  {
    std::cerr << "Failed to create feature" << std::endl;
    return;
  }

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
void Postprocessor::simplify()
{
  int oldSize = items.size();

  std::vector<Node> newItems = std::vector<Node>();
  // newItems.push_back(items[0]);
  // for (int i = 1; i < items.size() - 1; i++)
  // {
  //   int dx1 = items[i].x - items[i - 1].x;
  //   int dy1 = items[i].y - items[i - 1].y;
  //   int dx2 = items[i + 1].x - items[i].x;
  //   int dy2 = items[i + 1].y - items[i].y;

  //   if (abs(dx1 * dy2) * abs(dx2 * dy1) != 0)
  //   {
  //     newItems.push_back(items[i]);
  //   }
  // }
  // newItems.push_back(items[items.size() - 1]);
  // items = newItems;
  newItems = this->douglasPeucker(items.begin(), items.end() - 1, 5.5);
  items = newItems;

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