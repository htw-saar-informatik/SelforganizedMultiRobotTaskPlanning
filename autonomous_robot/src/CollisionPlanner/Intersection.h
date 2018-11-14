#ifndef INTERSECTION_H_
#define INTERSECTION_H_

// class Segment;

#include <geometry_msgs/Point.h>
#include "Line.h"


class Intersection {
public:
    enum EType { aPlus, aMinus, bPlus, bMinus };
	Intersection();
    Intersection( geometry_msgs::Point intersectionPoint,  Line firstLine,Line secondLine, EType type,int segmentID,int segmentIDRobotTwo);
	virtual ~Intersection();

    geometry_msgs::Point intersectionPoint; //Points where the intersection occurs
    Line firstLine; //intersectionline of first robot
    Line secondLine;    //intersectionline of second robot
    int segmentID; //ID of segment for first robot
    int segmentIDRobotTwo;  //ID for segment of robot two
    EType type;
};

#endif /* INTERSECTION_H_ */
