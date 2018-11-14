#ifndef SEGMENT_H_
#define SEGMENT_H_

#include "Line.h"
#include "geometry_msgs/Point.h"
#include "SubPathPoint.h"

/**
 * Represents a Segment, the path of a robot gets divided in segments
 */ 
class Segment {
public:
	Segment(Line lineLeft, Line lineRight, Line shortLineLeft, Line shortLineRight, int id, geometry_msgs::Point originalPointStart, geometry_msgs::Point originalPointEnd);
    Segment();
    Line lineLeft;
    Line lineRight;
    Line shortLineLeft;
    Line shortLineRight;
    geometry_msgs::Point originalPointStart;
    geometry_msgs::Point originalPointEnd;
    geometry_msgs::Point centeredStartPoint;    //centered point on the segment, behind de originalPoint
    geometry_msgs::Point centeredEndPoint;
    double getSegmentLenght();
    double getDistanceToPoint(geometry_msgs::Point point);
    std::vector<SubPathPoint> subPahtPoints;
    void addSubPathPoint(SubPathPoint point);
    int id;
    bool isPointInSegment(geometry_msgs::Point p);
    double getOrientation(Line line, geometry_msgs::Point point);
	virtual ~Segment();
};

#endif /* SEGMENT_H_ */
