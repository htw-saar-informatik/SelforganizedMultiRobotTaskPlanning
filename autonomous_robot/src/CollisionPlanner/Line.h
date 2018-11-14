#ifndef LINE_H_
#define LINE_H_
#include <cmath>    //pow,sqrt
#include <geometry_msgs/Point.h>

class Line {
public:
	Line();
	virtual ~Line();
      //Lines directs from firstPoint -> secondPoint
    geometry_msgs::Point firstPoint;
    geometry_msgs::Point secondPoint;

    Line(geometry_msgs::Point firstPoint,geometry_msgs::Point secondPoint);

    //Checks if x and y koordinates for first and secondpoint are equal
    bool linesEqual(Line& line);
    double getLength();

    static bool doLinesIntersect(Line a, Line b);
    static Line getIntersection(Line a, Line b);

private:
     //Methods for line intersections
    static std::vector<geometry_msgs::Point> getBoundingBox(geometry_msgs::Point first, geometry_msgs::Point second);
    static bool doBoundingBoxesIntersect(std::vector<geometry_msgs::Point> a, std::vector<geometry_msgs::Point> b);
    static bool lineSegmentTouchesOrCrossesLine(Line a, Line b);
    static bool isPointOnLine(Line a, geometry_msgs::Point b);
    static double crossProduct(geometry_msgs::Point a, geometry_msgs::Point b);
    static double EPSILON;
    static bool xOrOperator(bool a, bool b);
    static bool isPointRightOfLine(Line a, geometry_msgs::Point b);
    static bool compareDoubles(double a, double b);
};

#endif /* LINE_H_ */
