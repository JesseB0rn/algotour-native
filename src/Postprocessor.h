#ifndef POSTPROCESSOR_H
#define POSTPROCESSOR_H

#include <vector>
#include <tuple>
#include "QueueItem.h"
#include "GeoTiffLoader.h"

class Postprocessor
{
public:
  static void writeReprojectedGeoJSON(const std::vector<std::tuple<double, double>> &path, const char *filename);
  static void simplify(std::vector<Node> &items);
  static std::vector<std::tuple<double, double>> smooth(std::vector<Node> &items, GeoTiffLoader *riskmap);
  static double distanceToLine(Node pA, Node pTest, Node pB);
  static std::vector<Node> douglasPeucker(std::vector<Node>::iterator start, std::vector<Node>::iterator end, double epsilon);

private:
  // std::vector<Node> items;
  // std::vector<std::tuple<double, double>> smoothPath;
};

#endif // POSTPROCESSOR_H