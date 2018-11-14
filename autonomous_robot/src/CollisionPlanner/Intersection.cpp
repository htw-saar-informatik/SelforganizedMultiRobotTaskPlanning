#include "Intersection.h"

Intersection::Intersection() {
	// TODO Auto-generated constructor stub

}

Intersection::Intersection( geometry_msgs::Point intersectionPoint,  Line firstLine,Line secondLine, EType type,int segmentID,int segmentIDRobotTwo){
    this->intersectionPoint = intersectionPoint;
    this-> firstLine = firstLine;
    this->secondLine = secondLine;
    this->type = type;
    this->segmentID = segmentID;
    this->segmentIDRobotTwo = segmentIDRobotTwo;
}

Intersection::~Intersection() {
	// TODO Auto-generated destructor stub
}

