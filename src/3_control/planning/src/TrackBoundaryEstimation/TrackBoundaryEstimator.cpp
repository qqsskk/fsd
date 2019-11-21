#include "TrackBoundaryEstimator.hpp"
#include <vector>
#include <math.h>
#include "objects/Line.hpp"
#include "objects/Point.hpp"
#include "objects/Cone.hpp"

#include <algorithm>
#include <iostream>

typedef std::vector<Cone> Cones;
typedef std::vector<Line> Lines;

TrackBoundary TrackBoundaryEstimator::connect(int connection, std::vector<Cone> blueCones, std::vector<Cone> yellowCones, std::vector<Cone> orangeCones) {
    switch(connection) {
        case TrackBoundaryEstimator::CONNECTION_FULL: {
            return fullyConnection(blueCones, yellowCones, orangeCones);
        }
        case TrackBoundaryEstimator::CONNECTION_SNAKE: {
            return snakeTrack(blueCones, yellowCones, orangeCones);
        }
        case TrackBoundaryEstimator::CONNECTION_ACCELERATION: {
            return connectAcceleration(blueCones, yellowCones, orangeCones);
        }
        case TrackBoundaryEstimator::CONNECTION_SKID_PAD: {
            return connectSkidPad(blueCones, yellowCones, orangeCones);
        }
    }
    TrackBoundary tb;
    return tb;
}

//Called to connect the skid pad track
TrackBoundary TrackBoundaryEstimator::connectSkidPad(Cones blueCones, Cones yellowCones, Cones orangeCones) {
    TrackBoundary tb;
    //Connect coloured cones
    for (Line line : skidPadConnect(blueCones))
        tb.blueLines.push_back(line);
    for (Line line : skidPadConnect(yellowCones))
        tb.yellowLines.push_back(line);
    
    //Connect orange cones
    Cones allCones;
    //Copy coloured cones into all cones
    for (Cone c : orangeCones)
        allCones.push_back(c);
    for (Cone c : blueCones)
        allCones.push_back(c);
    for (Cone c : yellowCones)
        allCones.push_back(c);
    
    //Closest connect orange
    for (Cone c : orangeCones) {
        //Create Lines
        Cones conesCopy;
        for (Cone ac : allCones)
            if (c != ac)
                conesCopy.push_back(ac);
        
        Cones bcs = getClosestCones(c.pos, conesCopy, 2);
        
        double angleBetweenCones = findAngle(bcs[0], c, bcs[1]);
        if (sin(angleBetweenCones) < 0.90) {
            tb.orangeLines.push_back(c.pos.lineTo(bcs[0].pos));
            tb.orangeLines.push_back(c.pos.lineTo(bcs[1].pos));
        }
    }
    return tb;
}

//Called to connected coloured cones
Lines TrackBoundaryEstimator::skidPadConnect(Cones cones) {
    Lines lines;
    for (Cone c : cones) {
        //Create Lines
        Cones conesCopy;
        for (Cone ac : cones)
            if (c != ac)
                conesCopy.push_back(ac);
        
        Cones bcs = getClosestCones(c.pos, conesCopy, 2);
        Line l1 = c.pos.lineTo(bcs[0].pos); double l1l = l1.length();
        Line l2 = c.pos.lineTo(bcs[1].pos); double l2l = l2.length();
        double ratio = l1l / l2l; if (ratio > 1) {ratio = 1 / ratio;}
        
        if (ratio > 0.9) {
            lines.push_back(l1);
            lines.push_back(l2);
        } else if (l1l < l2l) {
            lines.push_back(l1);
        } else {
            lines.push_back(l2);
        }
    }
    return lines;
}
    
TrackBoundary TrackBoundaryEstimator::snakeTrack(Cones blueCones, Cones yellowCones, Cones orangeCones) {
    TrackBoundary tb;
    
    //Buffer for cones
    Cones blueConesCopy(blueCones);
    Cones yellowConesCopy(yellowCones);
    
    Cones allCones(blueConesCopy);
    for (Cone c : yellowConesCopy)
        allCones.push_back(c);
    
    for (Cone cone : orangeCones) {
        Cones closest = getClosestCones(cone.pos, allCones, 3);
        
        //create lines and get majority
        int blue = 0, yellow = 0;
        for (Cone c : closest) {
            if (c.colour == Cone::BLUE_CONE) {
                blue++;
            } else if (c.colour == Cone::YELLOW_CONE) {
                yellow++;
            }
        }
        
        //Create lines
        if (blue > yellow) {
            blueConesCopy.push_back(cone);
        } else {
            yellowConesCopy.push_back(cone);
        }
    }
    
    tb = snake(tb, blueConesCopy);
    tb = snake(tb, yellowConesCopy);
    return tb;
}
    
TrackBoundary TrackBoundaryEstimator::fullyConnection(Cones blueCones, Cones yellowCones, Cones orangeCones) {
    TrackBoundary tb;
    //Full connect main cones
    tb.blueLines = fullyConnect(blueCones);
    tb.yellowLines = fullyConnect(yellowCones);
    
    //Connect orange cones
    Cones allCones(blueCones);
    for (Cone c : yellowCones)
        allCones.push_back(c);
    for (Cone cone : orangeCones) {
        Cones closest = getClosestCones(cone.pos, allCones, 3);
        
        //create lines and get majority
        Lines blueLines;
        Lines yellowLines;
        for (Cone c : closest) {
            Line l = c.pos.lineTo(cone.pos);
            if (c.colour == Cone::BLUE_CONE) {
                blueLines.push_back(l);
            } else {
                yellowLines.push_back(l);
            }
        }
        
        //Create lines
        if (blueLines.size() > yellowLines.size()) {
            for (Line l : blueLines)
                tb.orangeLines.push_back(l);
        } else {
            for (Line l : yellowLines)
                tb.orangeLines.push_back(l);
        }
    }
    
    return tb;
}
    
Cones TrackBoundaryEstimator::getClosestCones(Point origin, Cones cones, int count) {
    Cones closestCones;
    for (Cone aC : cones) {
        bool added = false;
        for (int i = 0; i < fmin(3, closestCones.size()); i++) {
            if (origin.distanceTo(aC.pos) < origin.distanceTo(closestCones[i].pos)) {
                closestCones.insert(closestCones.begin() + i, aC);
                i = (int) closestCones.size();
                added = true;
            }
        }
        
        if (closestCones.size() < 3 && !added) {closestCones.push_back(aC);}
        while (closestCones.size() > 3) {closestCones.erase(closestCones.begin() + 3);}
    }
    return closestCones;
}
    
TrackBoundary TrackBoundaryEstimator::connectAcceleration(Cones blueCones, Cones yellowCones, Cones orangeCones) {
    TrackBoundary tb;
    Cones allCones(blueCones);
    for (Cone c : orangeCones)
        allCones.push_back(c);
    for (Cone c : yellowCones)
        allCones.push_back(c);
    return snake(tb, allCones);
}
    
TrackBoundary TrackBoundaryEstimator::snake(TrackBoundary tb, Cones cones) {
    //Constant
    int depth = 1;
    
    //Algorithm buffers
    Cones uncheckedCones(cones);
    if (uncheckedCones.size() > 1) {
        Cone joinedCone = uncheckedCones[0];
        Cone checkingCone = uncheckedCones[1];
        const Cone initalCone = uncheckedCones[1];
        bool searching = true;
        
        //Join Cones
        while (searching) {
            //Build buffer and initial snake
            std::vector<Cones> snakes;
            Cones originSnake; originSnake.push_back(joinedCone); originSnake.push_back(checkingCone); snakes.push_back(originSnake);
            
            for (int d = 0; d < depth; d++) {
                //New snakes buffer
                std::vector<Cones> newSnakes;
                for (int s = 0; s < snakes.size(); s++) {
                    //check how snake moves
                    for (int p = 0; p < uncheckedCones.size(); p++) {
                        //Create new snake
                        if (std::find(snakes[s].begin(), snakes[s].end(), uncheckedCones[p]) == snakes[s].end()) {
                            Cones snakeCopy(snakes[s]);
                            snakeCopy.push_back(uncheckedCones[p]);
                            newSnakes.push_back(snakeCopy);
                        }
                    }
                }
                snakes = newSnakes;
            }
            
            if (!snakes.empty()) {
                //check straightest snake
                Cones straightestSnake = snakes[0];
                double bestHeuristic = snakeHeuristic(straightestSnake);
                for (int s = 1; s < snakes.size(); s++) {
                    double huristic = snakeHeuristic(snakes[s]);
                    if (huristic < bestHeuristic) {
                        bestHeuristic = huristic;
                        straightestSnake = snakes[s];
                    }
                }
                
                //Create Line From Snake
                tb = createSnakeLine(tb, straightestSnake[1], straightestSnake[2]);
                
                checkingCone = straightestSnake[2];
                joinedCone = straightestSnake[1];
                
                //uncheckedCones.remove(straightestSnake[1]);
                uncheckedCones.erase(std::find(uncheckedCones.begin(), uncheckedCones.end(), straightestSnake[1]));
                
            } else {
                searching = false;
            }
        }
        //Join last cones
        return createSnakeLine(tb, initalCone, uncheckedCones[0]);
    }
    return tb;
}
    
TrackBoundary TrackBoundaryEstimator::createSnakeLine(TrackBoundary tb, Cone a, Cone b) {
    Line line(a.pos, b.pos);
    if (line.length() < 2 * MAX_TRACK_BOUNDARY_DISTANCE) {
        if (a.colour == Cone::ORANGE_CONE || b.colour == Cone::ORANGE_CONE) {
            //Orange Cone
            tb.orangeLines.push_back(line);
        } else if (a.colour == Cone::BLUE_CONE) {
            //blue cone
            tb.blueLines.push_back(line);
        } else {
            //yellow cone
            tb.yellowLines.push_back(line);
        }
    } else {
        //Will need to do nearest neighbour
    }
    return tb;
}
    

//Snake length must be >= 3
double TrackBoundaryEstimator::snakeHeuristic(Cones snake) {
    double totalHeiristic = 0;
    
    for (int h = 0; h < snake.size() - 2; h++) {
        Cone p1 = snake[h], p2 = snake[h+1], p3 = snake[h+2];
        double angle = findAngle(p1, p2, p3);
        totalHeiristic += angleBias * abs(M_PI - angle);
    }
    
    for (int h = 0; h < snake.size() - 1; h++) {
        double dX = snake[h].pos.x - snake[h+1].pos.x, dY = snake[h].pos.y - snake[h+1].pos.y;
        totalHeiristic += lengthBias * hypot(dX, dY);
    }
    return totalHeiristic;
}
    
/**
 * Get angle between a-c at b
 *
 * @param A First Cone
 * @param B Second Cone, find angle between
 * @param C Third Cone
 * @return Return angle at B between A & C
 */
double TrackBoundaryEstimator::findAngle(Cone A, Cone B, Cone C) {
    double AB = sqrt(pow(B.pos.x - A.pos.x, 2) + pow(B.pos.y - A.pos.y, 2));
    double BC = sqrt(pow(B.pos.x - C.pos.x, 2) + pow(B.pos.y - C.pos.y, 2));
    double AC = sqrt(pow(C.pos.x - A.pos.x, 2) + pow(C.pos.y - A.pos.y, 2));
    double ogAngle = (BC*BC + AB*AB - AC*AC) / (2 * BC*AB);
    ogAngle = fmin(fmax(ogAngle, -1), 1);
    return acos(ogAngle);
}

/**
 * Fully connect all cones given
 *
 * @param cones First Cone
 * @return Return List of lines fully connecting the given cones
 */
Lines TrackBoundaryEstimator::fullyConnect(Cones cones) {
    //Set up array buffers
    Cones colouredCones(cones);
    Lines trackBoundaries;
    
    //Fully Connected
    if (colouredCones.size() > 0) {
        for (int s = 0; s < colouredCones.size() - 1; s++) {
            Cone start = colouredCones[s];
            for (int e = s + 1; e < colouredCones.size(); e++) {
                Cone end = colouredCones[e];
                if (start.pos.distanceTo(end.pos) < MAX_TRACK_BOUNDARY_DISTANCE) {
                    Line l(start.pos, end.pos);
                    trackBoundaries.push_back(l);
                }
            }
        }
    }
    return trackBoundaries;
}
