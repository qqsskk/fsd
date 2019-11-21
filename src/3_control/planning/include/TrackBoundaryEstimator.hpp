//
//  TrackBoundary.hpp
//  TrackBoundaryEstimation
//
//  Created by Sam Garlick on 06/06/2019.
//  Copyright © 2019 FS-AI. All rights reserved.
//

#ifndef TrackBoundaryEstimator_hpp
#define TrackBoundaryEstimator_hpp

#include <stdio.h>
#include <vector>
#include "objects/Cone.hpp"
#include "objects/Line.hpp"

struct TrackBoundary {
    std::vector<Line> blueLines;
    std::vector<Line> yellowLines;
    std::vector<Line> orangeLines;
};

class TrackBoundaryEstimator {
public:
    //Public connection modes - detemined by AI...
    static const int CONNECTION_SNAKE = 0,
                     CONNECTION_FULL = 1,
                     CONNECTION_ACCELERATION = 2,
                     CONNECTION_SKID_PAD = 3;
    
    //Max fully connection distance
    static constexpr double MAX_TRACK_BOUNDARY_DISTANCE = 6;
    
    //public connect void
    static TrackBoundary connect(int connection, std::vector<Cone> blueCones,
                                 std::vector<Cone> yellowCones,
                                 std::vector<Cone> orangeCones);
    

private:
    //Snake heuristic bias
    static constexpr double angleBias = 2;
    static constexpr double lengthBias = 1;
    
    static TrackBoundary connectSkidPad(std::vector<Cone> blueCones, std::vector<Cone> yellowCones, std::vector<Cone> orangeCones);
    static std::vector<Line> skidPadConnect(std::vector<Cone> cones);
    
    static TrackBoundary fullyConnection(std::vector<Cone> blueCones, std::vector<Cone> yellowCones, std::vector<Cone> orangeCones);
    static std::vector<Cone> getClosestCones(Point origin, std::vector<Cone> cones, int count);
  
    static TrackBoundary snakeTrack(std::vector<Cone> blueCones, std::vector<Cone> yellowCones, std::vector<Cone> orangeCones);
    
    static TrackBoundary connectAcceleration(std::vector<Cone> blueCones, std::vector<Cone> yellowCones, std::vector<Cone> orangeCones);
    static TrackBoundary snake(TrackBoundary tbl, std::vector<Cone> cones);
    static TrackBoundary createSnakeLine(TrackBoundary tbl, Cone a, Cone b);

    //Snake length must be >= 3
    static double snakeHeuristic(std::vector<Cone> snake);
    static double findAngle(Cone A, Cone B, Cone C);
    static std::vector<Line> fullyConnect(std::vector<Cone> cones);
};



#endif /* TrackBoundary_hpp */
