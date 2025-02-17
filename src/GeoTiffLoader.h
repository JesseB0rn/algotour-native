#ifndef GEOTIFFLOADER_H
#define GEOTIFFLOADER_H

#define NODATA_VALUE -9999
#define INTERNAL_NODATA_VALUE -9999

#include <string>
#include <gdal.h>
#include <gdal_priv.h>

class GeoTiffLoader
{
public:
    GeoTiffLoader(const char *filePath);
    ~GeoTiffLoader();

    float GetValueFromDatasetBuffer(double lat, double lon);
    float *GetVRefFromDatasetBuffer(double lat, double lon);
    float *GetVRefFromDatasetBuffer(int x, int y);
    void convertLatLonToPixel(double lat, double lon, int &x, int &y);
    void convertPixelToLatLon(int x, int y, double &lat, double &lon);

    int GetNXSize() { return nXSize; }
    int GetNYSize() { return nYSize; }

    GDALDataset *dataset;
private:
    float *buffer;
    double fwdTransform[6];
    int nXSize;
    int nYSize;

    bool LoadGeoTiff(const char *filePath);
};

#endif // GEOTIFFLOADER_H