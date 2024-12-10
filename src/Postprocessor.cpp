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
void Postprocessor::simplify()
{
  // Simplify the path
  // if the curvature is not larger than a certain threshold, do not copy the point to the new path

  int oldSize = items.size();

  std::vector<Node> newItems = std::vector<Node>();
  newItems.push_back(items[0]);
  for (int i = 1; i < items.size() - 1; i++)
  {
    int dx1 = items[i].x - items[i - 1].x;
    int dy1 = items[i].y - items[i - 1].y;
    int dx2 = items[i + 1].x - items[i].x;
    int dy2 = items[i + 1].y - items[i].y;

    if (abs(dx1 * dy2) * abs(dx2 * dy1) != 0)
    {
      newItems.push_back(items[i]);
    }
  }
  newItems.push_back(items[items.size() - 1]);
  items = newItems;

  int newSize = items.size();
  std::cout << "Simplified path from " << oldSize << " to " << newSize << " points" << std::endl;
}