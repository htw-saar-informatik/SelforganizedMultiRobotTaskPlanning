#ifndef SUBPATHPOINT_H_
#define SUBPATHPOINT_H_

#include "geometry_msgs/Point.h"
#include "LineElement.h"

/**
 * Represent a point on the path of robotOne, who is either the beginning of a subpath or the ending of a subpath
 */ 
class SubPathPoint {
public:
    enum SubPathType {beginSubPath, endSubPath};
	SubPathPoint();
    SubPathPoint(geometry_msgs::Point intersectionPoint,LineElement firstIntersection, LineElement secondIntersection, SubPathType type,int firstIntersectionSegmentId, int secondIntersectionSegmentId);
	virtual ~SubPathPoint();

    //The point in the path for robot one, either the start or the endposition of the subpath
    geometry_msgs::Point p; 
    LineElement firstIntersection;  //First intersection on the line of the contur from robot2
    LineElement secondIntersection; //Second intersection on the line of the contur from robot2
    int firstIntersectionSegmentId;
    int secondIntersectionSegmentId;

    SubPathType type;
};

#endif /* SUBPATHPOINT_H_ */
