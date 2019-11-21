#include "Point.hpp"
#include "Line.hpp"
#include <math.h>

//Constructors
Point::Point() : x(0), y(0) {}
Point::Point(double _x, double _y) : x(_x), y(_y) {}

Line Point::lineTo(Point b) {
    Point self(x, y);
    Line l(self, b);
    return l;
}

double Point::distanceTo(Point b) {
    return hypot(x - b.x, y - b.y);
}
