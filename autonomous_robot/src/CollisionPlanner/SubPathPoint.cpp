#include "SubPathPoint.h"

SubPathPoint::SubPathPoint() {
	// TODO Auto-generated constructor stub

}

SubPathPoint::~SubPathPoint() {
	// TODO Auto-generated destructor stub
}

 SubPathPoint::SubPathPoint(geometry_msgs::Point intersectionPoint, LineElement firstIntersection, LineElement secondIntersection, SubPathType type,int firstIntersectionSegmentId, int secondIntersectionSegmentId){
    this->p = intersectionPoint;
    this->firstIntersection = firstIntersection;
    this->secondIntersection = secondIntersection;
    this->type = type;
    this->firstIntersectionSegmentId = firstIntersectionSegmentId;
    this->secondIntersectionSegmentId = secondIntersectionSegmentId;
 }
