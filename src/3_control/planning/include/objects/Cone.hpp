//
//  Cone.hpp
//  TrackBoundaryEstimation
//
//  Created by Sam Garlick on 06/06/2019.
//  Copyright © 2019 FS-AI. All rights reserved.
//

#ifndef Cone_hpp
#define Cone_hpp

#include <stdio.h>
#include "Point.hpp"

class Cone {
public:
    static const int BLUE_CONE = 2, YELLOW_CONE = 1, ORANGE_CONE = 2;
    
    Point pos;
    int colour;
    
    Cone(double x, double y, int c);
    Cone(Point p, int c);
};

bool operator!=(const Cone a, const Cone b);
bool operator==(const Cone a, const Cone b);

#endif /* Cone_hpp */
