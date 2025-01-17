#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include "QueueItem.h"

class Helpers
{
public:
  static double distanceToLineBoundsCheck(Node pA, Node pTest, Node pB)
  {
    double A = pTest.x - pA.x;
    double B = pTest.y - pA.y;
    double C = pB.x - pA.x;
    double D = pB.y - pA.y;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = (len_sq != 0) ? dot / len_sq : -1;

    double xx, yy;

    if (param < 0)
    {
      xx = pA.x;
      yy = pA.y;
    }
    else if (param > 1)
    {
      xx = pB.x;
      yy = pB.y;
    }
    else
    {
      xx = pA.x + param * C;
      yy = pA.y + param * D;
    }

    double dx = pTest.x - xx;
    double dy = pTest.y - yy;
    return sqrt(dx * dx + dy * dy);
  }
};

#endif // HELPERS_H