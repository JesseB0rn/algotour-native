#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

#include <vector>
#include "QueueItem.h"
#include "GeoTiffLoader.h"

class Postprocessor
{
public:
  Postprocessor(const std::vector<Node> &items);
  void writeReprojectedGeoJSON(const char *filename, GeoTiffLoader *riskmap);
  void simplify();

private:
  std::vector<Node> items;
  double distanceToLine(Node pA, Node pTest, Node pB);
  std::vector<Node> douglasPeucker(std::vector<Node>::iterator start, std::vector<Node>::iterator end, double epsilon);
};

#endif // POSTPROCESSOR_H