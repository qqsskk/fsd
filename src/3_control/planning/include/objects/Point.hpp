//
//  Point.hpp
//  TrackBoundaryEstimation
//
//  Created by Sam Garlick on 06/06/2019.
//  Copyright © 2019 FS-AI. All rights reserved.
//

#ifndef Point_hpp
#define Point_hpp

#include <stdio.h>
#include "Line.hpp"

class Point {
public:
    double x, y;
    
    Point();
    Point(double x, double y);
    
    Line lineTo(Point b);
    double distanceTo(Point b);
};

#endif /* Point_hpp */
