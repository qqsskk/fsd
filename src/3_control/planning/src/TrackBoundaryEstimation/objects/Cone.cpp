#include "Cone.hpp"

//Constructors
Cone::Cone(double x, double y, int c) : pos(x, y), colour(c) {}
Cone::Cone(Point _p, int c) : pos(_p), colour(c) {}

bool operator!=(const Cone a, const Cone b) {
    return !(a==b);
}

bool operator==(const Cone a, const Cone b) {
    return a.pos.x == b.pos.x && a.pos.y == b.pos.y && a.colour == b.colour;
}

