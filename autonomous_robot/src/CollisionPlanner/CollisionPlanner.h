#ifndef COLLISIONPLANNER_H_
#define COLLISIONPLANNER_H_


#include "Robot.h"
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <cmath>    //pow,sqrt
#include "Intersection.h"
#include "Line.h"
#include <algorithm>    // std::min
#include "LineElement.h"
#include "Segment.h"
#include "SubPathPoint.h"
#include "CollisionPath.h"
class Collision{
public:
    geometry_msgs::Point waitPoint;
    geometry_msgs::Point reachPoint;
};

struct CollisionResult{
    std::vector<Collision> collisions;
    bool isValid;   //Has an error occured?
    bool canDrive;  //Is it possible for the robot to drive?
};

class Rectangle{
public:
    geometry_msgs::Point location;  //Uper left corner position for the rectangel
    double width;
    double height;
};


class CollisionPlanner {
public:
	CollisionPlanner();
	virtual ~CollisionPlanner();
    //pointsPathOne = Path for first robot who whats to calculate the collisions with the path pointsPathTwo from a second robot
    CollisionResult calculateCollisions(std::vector<geometry_msgs::Point> pointsPathOne, std::vector<geometry_msgs::Point> pointsPathTwo, int radiusRobotOne, int radiusRobotTwo);
   
    std::vector<CollisionPath> getCollisionPaths(Robot robotOne, Robot robotTwo);

private:
   
    std::map<int,std::vector<Intersection> > map;
    //Returns the collisionSubPaths on the path of robotOne with the Contur of robotTwo
    double getOrientation(Line line, geometry_msgs::Point point);
    void addToMap(int index, Intersection intersection);
    double getDistanceBetweenPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);
    void calculateIntersectionsOnContur(Robot robotOne, std::vector<Line> contur,int countSegmentsRobotTwo);
    double getDistanceOnPathforIntersection(Intersection intersection, std::vector<Segment> segmentList);
    std::vector<CollisionPath> calculateSubPaths(std::vector<LineElement> lineElements, std::vector<Segment> segmentList);
    bool containsPoint(std::vector<geometry_msgs::Point> points, geometry_msgs::Point point);
    bool checkIfRectangleAndLineIntersect(Rectangle& rect, Line& line);
    // bool checkIfSegmentAndLineIntersect(Rectangle& rect, Line& line);
    bool compareDoubles(double a, double b);
    bool checkIfPointIsOnWay(std::vector<Segment> segments, geometry_msgs::Point p,double radius);
   };

#endif /* COLLISIONPLANNER_H_ */
