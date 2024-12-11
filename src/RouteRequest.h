#ifndef ROUTEREQ_H
#define ROUTEREQ_H

#include <tuple>
#include <vector>
#include "Postprocessor.h"

class RouteRequest
{
public:
  RouteRequest(std::tuple<double, double> start, std::tuple<double, double> end, GeoTiffLoader &riskmap, GeoTiffLoader &dem);
  void run();

private:
  std::tuple<double, double> start;
  std::tuple<double, double> end;
  GeoTiffLoader &riskmap;
  GeoTiffLoader &dem;
  char *filename;

  std::vector<Node> runWalkOnRasters();
  float walk_time_cost(float demValueA, float demValueB, float distance);
};

#endif // ROUTEREQ_H