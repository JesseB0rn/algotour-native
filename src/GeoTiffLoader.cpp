#include "GeoTiffLoader.h"
#include <iostream>

GeoTiffLoader::GeoTiffLoader(const char *filePath) : dataset(nullptr), buffer(nullptr)
{
  if (!LoadGeoTiff(filePath))
  {
    std::cerr << "Failed to load GeoTIFF file." << std::endl;
  };
}

GeoTiffLoader::~GeoTiffLoader()
{
  if (buffer)
  {
    CPLFree(buffer);
  }
  if (dataset)
  {
    GDALClose(dataset);
  }
}

bool GeoTiffLoader::LoadGeoTiff(const char *filePath)
{
  dataset = (GDALDataset *)GDALOpen(filePath, GA_ReadOnly);
  if (dataset == nullptr)
  {
    std::cerr << "Failed to open GeoTIFF file." << std::endl;
    return false;
  }

  dataset->GetGeoTransform(fwdTransform);
  nXSize = dataset->GetRasterXSize();
  nYSize = dataset->GetRasterYSize();

  GDALRasterBand *band = dataset->GetRasterBand(1);
  buffer = (float *)CPLMalloc(sizeof(float) * nXSize * nYSize);
  auto err = band->RasterIO(GF_Read, 0, 0, nXSize, nYSize, buffer, nXSize, nYSize, GDT_Float32, 0, 0);
  if (err != CE_None)
  {
    std::cerr << "Failed to read raster data." << std::endl;
    return false;
  }
  return true;
}

// NOTE: Double fwd Transform is intentional because of weirdness suroundoing the LV95 projection (epsg:2056)
//       and the way QGIS writes the corresponding geotransform

/**
 * @brief Get the Value From Dataset Buffer object at the given real World lat and lon
 *
 * @param lat LV03+ latitude
 * @param lon LV03+ longitude
 * @return float
 */
float GeoTiffLoader::GetValueFromDatasetBuffer(double lat, double lon)
{
  int pixelY = (lat - fwdTransform[0]) / fwdTransform[1];
  int pixelX = (lon - fwdTransform[3]) / fwdTransform[5];

  if (pixelX < 0 || pixelX >= nXSize || pixelY < 0 || pixelY >= nYSize)
  {
    return NODATA_VALUE;
  }

  return buffer[pixelY * nXSize + pixelX];
}

/**
 * @brief Get the pointer to value at the given real World lat and lon from the dataset buffer
 *
 * @param lat LV03+ latitude
 * @param lon LV03+ longitude
 * @return float*
 */
float *GeoTiffLoader::GetVRefFromDatasetBuffer(double lat, double lon)
{
  int pixelY = (lat - fwdTransform[0]) / fwdTransform[1];
  int pixelX = (lon - fwdTransform[3]) / fwdTransform[5];

  if (pixelX < 0 || pixelX >= nXSize || pixelY < 0 || pixelY >= nYSize)
  {
    return nullptr;
  }

  return &buffer[pixelY * nXSize + pixelX];
}

/**
 * @brief Get the pointer to value at the given pixel coordinates from the dataset buffer
 *
 * @param x pixel x coordinate
 * @param  y pixel y coordinate
 * @return float*
 */
float *GeoTiffLoader::GetVRefFromDatasetBuffer(int x, int y)
{
  if (x < 0 || x >= nXSize || y < 0 || y >= nYSize)
  {
    return nullptr;
  }

  return &buffer[y * nXSize + x];
}

/**
 * @brief Convert the given lat and lon to pixel coordinates
 *
 * @param lat LV03+ latitude
 * @param lon LV03+ longitude
 * @param x pixel x coordinate
 * @param y pixel y coordinate
 */
void GeoTiffLoader::convertLatLonToPixel(double lat, double lon, int &x, int &y)
{
  y = (lat - fwdTransform[0]) / fwdTransform[1];
  x = (lon - fwdTransform[3]) / fwdTransform[5];
}

/**
 * @brief Convert the given pixel coordinates to lat and lon
 *
 * @param x pixel x coordinate
 * @param y pixel y coordinate
 * @param lat LV03+ latitude
 * @param lon LV03+ longitude
 */
void GeoTiffLoader::convertPixelToLatLon(int x, int y, double &lat, double &lon)
{
  lat = fwdTransform[0] + y * fwdTransform[1];
  lon = fwdTransform[3] + x * fwdTransform[5];
}
