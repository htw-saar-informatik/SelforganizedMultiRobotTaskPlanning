#ifndef ROBOT_H_
#define ROBOT_H_

#include "geometry_msgs/Point.h"
// #include "CollisionPlanner.h"
#include "visualization_msgs/Marker.h"
#include "../SharedFunctions.h"
#include "LinearFunction.h"
#include "Line.h"
#include "Segment.h"
#include "ros/ros.h"
#include <chrono>
#include <thread>
#include <algorithm>
#include <math.h> //fabs, tan, pi
#include <map>


class Robot {
public:
	Robot(int radius, std::vector<geometry_msgs::Point> pointsPath);
	virtual ~Robot();

	//Calculates the trace for the robot. Has to be calles before another function is called
	void setupRobot();
	std::vector<Line> getContur();
	void sleep(int millis);
	std::vector<Line> originalTraceLeft;
	std::vector<Line> originalTraceRight;
	std::vector<Line> smoothedTraceLeft;
	std::vector<Line> smoothedTraceRight;
	std::vector<Segment> segments;
	std::vector<geometry_msgs::Point> pointsPath;
	int radius;
	void printData();
private:
	
	std::vector<Line> calculateTraceForPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);
	void drawLine(Line& l);	//Draws a line to rviz
	void setMarker(geometry_msgs::Point p);
	std::vector<geometry_msgs::Point> calculatePointsOnCircle(geometry_msgs::Point p1, geometry_msgs::Point p2, bool outgoingPoint);
	LinearFunction calculateLinearFunction(geometry_msgs::Point p1, geometry_msgs::Point p2);
	LinearFunction calculateOrthogonalFunctionForPoint(LinearFunction& lf, geometry_msgs::Point p);
	void smoothLines(std::vector<Line>& lines);
	
};

#endif /* ROBOT_H_ */
