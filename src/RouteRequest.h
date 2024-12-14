#ifndef ROUTEREQ_H
#define ROUTEREQ_H

#include <tuple>
#include <vector>
#include "Postprocessor.h"

enum RouteRequestStatus
{
  SUCCESS = 0,
  FAILURE_OUT_OF_BOUNDS = -1,
  FAILURE_NO_PATH = -2,
  FAILURE_UNKNOWN = -3,
  FAILURE_USER_IS_A_DICK = -4
};

class RouteRequest
{
public:
  RouteRequest(std::tuple<double, double> start, std::tuple<double, double> end, GeoTiffLoader &riskmap, GeoTiffLoader &dem);
  RouteRequestStatus run(std::string &filename, double *progress = nullptr);

private:
  std::tuple<double, double> start;
  std::tuple<double, double> end;
  GeoTiffLoader &riskmap;
  GeoTiffLoader &dem;
  char *filename;

  std::vector<Node> runWalkOnRasters(RouteRequestStatus &status, double *progress = nullptr);
  float walk_time_cost(float demValueA, float demValueB, float distance);
};

#endif // ROUTEREQ_H