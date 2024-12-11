#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

#include <vector>
#include <tuple>
#include "QueueItem.h"
#include "GeoTiffLoader.h"

class Postprocessor
{
public:
  Postprocessor(const std::vector<Node> &items);
  void writeReprojectedGeoJSON(const char *filename);
  void simplify();
  void smooth(GeoTiffLoader *riskmap);

private:
  std::vector<Node> items;
  std::vector<std::tuple<double, double>> smoothPath;
  double distanceToLine(Node pA, Node pTest, Node pB);
  std::vector<Node> douglasPeucker(std::vector<Node>::iterator start, std::vector<Node>::iterator end, double epsilon);
};

#endif // POSTPROCESSOR_H