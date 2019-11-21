//
//  Line.hpp
//  TrackBoundaryEstimation
//
//  Created by Sam Garlick on 06/06/2019.
//  Copyright © 2019 FS-AI. All rights reserved.
//

#ifndef Line_hpp
#define Line_hpp

#include <stdio.h>

class Point;
class Cone;

class Line {
public:
    double ax, ay;
    double bx, by;
    
    Line(Point _a, Point _b);
    Line(Cone _a, Cone _b);
    
    double length();
};

#endif /* Line_hpp */
