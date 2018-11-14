#include "Segment.h"

Segment::Segment(Line lineLeft, Line lineRight, Line shortLineLeft, Line shortLineRight, int id, geometry_msgs::Point originalPointStart, geometry_msgs::Point originalPointEnd){
	this->lineLeft = lineLeft;
    this->lineRight = lineRight;
    this->shortLineLeft = shortLineLeft;
    this->shortLineRight = shortLineRight;
    this->id = id;
    this->originalPointEnd = originalPointEnd;
    this->originalPointStart = originalPointStart;
    geometry_msgs::Point csp;
    csp.x = (lineLeft.firstPoint.x + lineRight.firstPoint.x)/2;
    csp.y = (lineLeft.firstPoint.y + lineRight.firstPoint.y)/2;
    geometry_msgs::Point cep;
    cep.x = (lineLeft.secondPoint.x + lineRight.secondPoint.x) /2;
    cep.y = (lineLeft.secondPoint.y + lineRight.secondPoint.y) / 2;
    this->centeredStartPoint = csp;
    this->centeredEndPoint = cep;
}

Segment::Segment(){

}

Segment::~Segment() {
	// TODO Auto-generated destructor stub
}

double Segment::getSegmentLenght(){
    return Line(centeredStartPoint,centeredEndPoint).getLength();
}
double Segment::getDistanceToPoint(geometry_msgs::Point point){
    return Line(centeredStartPoint,point).getLength();
}

/**
 * Inserts a new subpahtpoint, so that all subpahtpoints in this segment are ordered by the distance to the centeredStartPoint
 */ 
void Segment::addSubPathPoint(SubPathPoint point){
    std::vector<SubPathPoint> tmpList = subPahtPoints;
    tmpList.push_back(point);
    subPahtPoints.clear();

    int index = 0;
    double distance;
    SubPathPoint element;
    SubPathPoint currentElement;

    while ( tmpList.size() > 0 ){
        distance = getDistanceToPoint(tmpList[0].p);
        element = tmpList[0];
        index = 0;

        for (int i = 0; i < tmpList.size(); i++){
            currentElement = tmpList[i];
            if( getDistanceToPoint( currentElement.p) < distance ){
                element = currentElement;
                distance = getDistanceToPoint(currentElement.p);
                index = i;
            }
        }
        subPahtPoints.push_back(element);
        tmpList.erase(tmpList.begin()+index);

    }
}

/**
 * Checks if a point is in this segment, by traversing the contur clockwise and check if the orientation for the point is always on the right
 */ 
 bool Segment::isPointInSegment(geometry_msgs::Point p){
     Line l1 = shortLineLeft;
     Line l2(shortLineLeft.secondPoint,shortLineRight.secondPoint);
     Line l3(shortLineRight.secondPoint,shortLineRight.firstPoint);
     Line l4(shortLineRight.firstPoint,shortLineLeft.firstPoint);

     double o1 = getOrientation(l1,p);
     double o2 = getOrientation(l2,p);
     double o3 = getOrientation(l3,p);
     double o4 = getOrientation(l4,p);

     return (o1 < 0) && (o2 < 0) && (o3 < 0) && (o4 < 0);
 }
 double Segment::getOrientation(Line line, geometry_msgs::Point point){
    return ( line.secondPoint.x - line.firstPoint.x) * (point.y - line.firstPoint.y) - (line.secondPoint.y - line.firstPoint.y) * (point.x - line.firstPoint.x);
}

