#ifndef QUEUEITM_H
#define QUEUEITM_H



typedef struct
{
  int x_riskmap;
  int y_riskmap;
  int x_dem;
  int y_dem;
  float totalCost;
} WalkQueueItem;

typedef struct
{
  int x;
  int y;
} WalkItem;

struct cmp_walkcost
{
  bool operator()(WalkQueueItem left, WalkQueueItem right)
  {
    return left.totalCost > right.totalCost;
  }
};

#endif // QUEUEITM_H