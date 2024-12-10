#include "QueueItem.h"

Node::Node(int _x, int _y)
    : x(_x), y(_y), f(0), g(0), h(0)
{
}

bool Node::operator>(const Node &other) const
{
  return f > other.f;
}

bool Node::operator==(const Node &other) const
{
  return x == other.x && y == other.y;
}