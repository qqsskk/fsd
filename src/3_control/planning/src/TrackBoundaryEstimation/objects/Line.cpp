#include "Line.hpp"
#include <math.h>
#include "Point.hpp"
#include "Cone.hpp"

Line::Line(Point _a, Point _b): ax(_a.x), ay(_a.y), bx(_b.x), by(_b.y) {}
Line::Line(Cone _a, Cone _b): ax(_a.pos.x), ay(_a.pos.y), bx(_b.pos.x), by(_b.pos.y) {}

double Line::length() {
    return hypot(ax - bx, ay - by);
}
