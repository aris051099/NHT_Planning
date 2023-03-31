#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

class Point {
public:
    Point(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}

    Point(const Point& other) : x(other.x), y(other.y), z(other.z), w(other.w) {}

    Point& operator=(const Point& other) {
        if (this == &other) {
            return *this;
        }

        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;

        return *this;
    }

    double x, y, z, w;
};

class KDNode 
{
public:
    Point point;
    KDNode* left;
    KDNode* right;
    int axis;

    KDNode(const Point& p, int axis) : point(p), left(nullptr), right(nullptr), axis(axis) {}
};


struct AxisComparator 
{
    explicit AxisComparator(int axis) : axis(axis) {}

    bool operator()(const Point& a, const Point& b) const {
        return (axis == 0) ? a.x < b.x :
               (axis == 1) ? a.y < b.y :
               (axis == 2) ? a.z < b.z :
                            a.w < b.w;
    }

    int axis;
};

class KDTree 
{
public:
    KDTree() : root(nullptr) {}

    void build(std::vector<Point>& points) {
        root = build(points.begin(), points.end(), 0);
    }

    Point nearest_neighbor(const Point& target) const {
        double min_distance = std::numeric_limits<double>::max();
        Point nearest_point = target;
        nearest_neighbor(root, target, min_distance, nearest_point);
        return nearest_point;
    }
private:

    KDNode* root;

    using PointIter = std::vector<Point>::iterator;

    KDNode* build(PointIter begin, PointIter end, int depth) 
    {
        if (begin == end) return nullptr;

        size_t size = std::distance(begin, end);
        int axis = depth % 4;

        AxisComparator comp(axis);

        std::nth_element(begin, begin + size / 2, end, comp);

        auto mid = begin + size / 2;
        KDNode* node = new KDNode(*mid, axis);
        node->left = build(begin, mid, depth + 1);
        node->right = build(mid + 1, end, depth + 1);

        return node;
    }


    double squared_distance(const Point& a, const Point& b) const {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        double dw = a.w - b.w;
        return dx * dx + dy * dy + dz * dz + dw * dw;
    }

    void nearest_neighbor(KDNode* node, const Point& target, double& min_distance, Point& nearest_point) const {
        if (!node) return;

        double distance = squared_distance(node->point, target);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_point = node->point;
        }

        double axis_diff = 0;
        int axis = node->axis;
        switch (axis) {
            case 0:
                axis_diff = target.x - node->point.x;
                break;
            case 1:
                axis_diff = target.y - node->point.y;
                break;
            case 2:
                axis_diff = target.z - node->point.z;
                break;
            case 3:
                axis_diff = target.w - node->point.w;
                break;
        }
        KDNode* first_child = axis_diff <= 0 ? node->left : node->right;
        KDNode* second_child = axis_diff <= 0 ? node->right : node->left;

        nearest_neighbor(first_child, target, min_distance, nearest_point);

        if (axis_diff * axis_diff < min_distance) {
            nearest_neighbor(second_child, target, min_distance, nearest_point);
        }
    }
};

void insert(KDNode* node, const Point& point, int depth) 
{
    if(node == nullptr)
    {
        std::cout<<"Root is null" << std::endl;
        return;
    }
    if (!node) {
        return;
    }

    int axis = depth % 4;
    bool goLeft = false;

    if (axis == 0) {
        goLeft = point.x < node->point.x;
    } else if (axis == 1) {
        goLeft = point.y < node->point.y;
    } else if (axis == 2) {
        goLeft = point.z < node->point.z;
    } else {
        goLeft = point.w < node->point.w;
    }

    KDNode*& nextNode = goLeft ? node->left : node->right;

    if (!nextNode) {
        nextNode = new KDNode(point, (depth + 1) % 4);
    } else {
        insert(nextNode, point, depth + 1);
    }
}

       
