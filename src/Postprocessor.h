#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

#include <vector>
#include "QueueItem.h"
#include "GeoTiffLoader.h"

class Postprocessor {
public:
  Postprocessor(const std::vector<WalkItem>& items);
  void writeReprojectedGeoJSON(const char *filename, GeoTiffLoader *riskmap);
  void simplify();

private:
  std::vector<WalkItem> items;
};

#endif // POSTPROCESSOR_H