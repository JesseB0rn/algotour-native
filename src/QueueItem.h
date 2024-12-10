#ifndef QUEUEITM_H
#define QUEUEITM_H



struct Node
{
    int x, y;
    int f, g, h;

    Node(int r, int c);

    bool operator>(const Node& other) const;
    bool operator==(const Node& other) const;
};

#endif // QUEUEITM_H