#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

#include <vector>
#include "QueueItem.h"
#include "GeoTiffLoader.h"

class Postprocessor {
public:
  Postprocessor(const std::vector<Node>& items);
  void writeReprojectedGeoJSON(const char *filename, GeoTiffLoader *riskmap);
  void simplify();

private:
  std::vector<Node> items;
};

#endif // POSTPROCESSOR_H