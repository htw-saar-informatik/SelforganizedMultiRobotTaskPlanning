#include "Line.h"

double Line::EPSILON = 0.000001;

Line::Line(){

}

Line::Line(geometry_msgs::Point firstPoint,geometry_msgs::Point secondPoint){
        this->firstPoint = firstPoint;
        this->secondPoint = secondPoint;
}

Line::~Line() {
	// TODO Auto-generated destructor stub
}

 //Checks if x and y koordinates for first and secondpoint are equal
    bool Line::linesEqual(Line& line){
        if( (line.firstPoint.x == this->firstPoint.x) && (line.firstPoint.y == this->firstPoint.y) ){
            if( (line.secondPoint.x == this->secondPoint.x) && (line.secondPoint.y == this->secondPoint.y) ){
                return true;
            }
        }
        return false;
    }

double Line::getLength(){
    return sqrt(  pow(secondPoint.y-firstPoint.y,2) + pow( secondPoint.x-firstPoint.x,2) );  
}

bool Line::doLinesIntersect(Line a, Line b){
    std::vector<geometry_msgs::Point> box1;
    std::vector<geometry_msgs::Point> box2;

    box1 = getBoundingBox(a.firstPoint,a.secondPoint);
    box2 = getBoundingBox(b.firstPoint,b.secondPoint);

    return doBoundingBoxesIntersect(box1,box2) && lineSegmentTouchesOrCrossesLine(a,b) && lineSegmentTouchesOrCrossesLine(b,a);
}

 Line Line::getIntersection(Line a, Line b){
     double x1, y1, x2, y2;

    // a.firstPoint.x == a.secondPoint.x 
     if ( Line::compareDoubles(a.firstPoint.x,a.secondPoint.x)){
         x1 = a.firstPoint.x;
         x2 = x1;

        // geometry_msgs::Point tmpPoint;
         if ( b.firstPoint.x == b.secondPoint.x ){
             if ( a.firstPoint.y > a.secondPoint.y ){
                a = Line(a.secondPoint,a.firstPoint);
             }

             if ( b.firstPoint.y > b.secondPoint.y ){
                 b = Line(b.secondPoint,b.firstPoint);
             }

             if( a.firstPoint.y > b.firstPoint.y ){
                Line tmpLine(a.firstPoint,a.secondPoint);
                a = b;
                b = tmpLine;
             }

             y1 = b.firstPoint.y;
             y2 = std::min(a.secondPoint.y,b.secondPoint.y);
         }else{
             double m,t;
             m = (b.firstPoint.y - b.secondPoint.y) / (b.firstPoint.x - b.secondPoint.x);
             t = b.firstPoint.y - m*b.firstPoint.x;
             y1 = m* x1+t;
             y2 = y1;
         }
         //b.firstPoint.x == b.secondPoint.x 
     }else if( Line::compareDoubles(b.firstPoint.x,b.secondPoint.x)){
        x1 = b.firstPoint.x;
        x2 = x1;

        Line tmpLine(a.firstPoint,a.secondPoint);
        a = b;
        b = tmpLine;

        double m,t;
        m = ( b.firstPoint.y - b.secondPoint.y ) / ( b.firstPoint.x - b.secondPoint.x );
        t = b.firstPoint.y - m*b.firstPoint.x;
        y1 = m*x1+t;
        y2 = y1;
    }else{
        double ma,mb,ta,tb;
        ma = ( a.firstPoint.y - a.secondPoint.y ) / ( a.firstPoint.x - a.secondPoint.x );
        mb = ( b.firstPoint.y - b.secondPoint.y ) / ( b.firstPoint.x - b.secondPoint.x );
        ta = a.firstPoint.y - ma*a.firstPoint.x;
        tb = b.firstPoint.y - mb*b.firstPoint.x;

        // ma = std::round(ma);
        // mb = std::round(mb);

        // if(  (ma != 1) && (ma != -1) ){
        //     std::cout << std::endl;
        // }
        // printf("ma %lf \n", ma);
        // printf("mb %lf \n", mb);

        //ma == mb
        if ( Line::compareDoubles(ma,mb) ){
            if( a.firstPoint.x > a.secondPoint.x ){
                a = Line(a.secondPoint,a.firstPoint);
            }
            if( b.firstPoint.x > b.secondPoint.x ){
                b = Line(b.secondPoint,b.firstPoint);
            }
            if( a.firstPoint.x > b.firstPoint.x ){
                Line tmpLine(a.firstPoint,a.secondPoint);
                a = b;
                b = tmpLine;
            }

            x1 = b.firstPoint.x;
            x2 = std::min(a.secondPoint.x,b.secondPoint.x);
            y1 = ma * x1 + ta;
            y2 = ma * x2 + ta;
        }else{
            x1 = (tb - ta) / (ma-mb);
            y1 = ma * x1 + ta;
            x2 = x1;
            y2 = y1;
        }
    }
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    p1.x = x1;
    p1.y = y1;
    p2.x = x2;
    p2.y = y2;
    return Line(p1,p2);
 }

std::vector<geometry_msgs::Point> Line::getBoundingBox(geometry_msgs::Point first, geometry_msgs::Point second){
    std::vector<geometry_msgs::Point> points;
    geometry_msgs::Point p1;
    p1.x = std::min(first.x,second.x);
    p1.y = std::min(first.y,second.y);

    geometry_msgs::Point p2;
    p2.x = std::max(first.x,second.x);
    p2.y = std::max(first.y,second.y);
    
    points.push_back(p1);
    points.push_back(p2);
    return points;
}

bool Line::doBoundingBoxesIntersect(std::vector<geometry_msgs::Point> a, std::vector<geometry_msgs::Point> b){
    return ( (a[0].x <= b[1].x) && ( a[1].x >= b[0].x ) && ( a[0].y <= b[1].y ) && ( a[1].y >= b[0].y) );
}

bool Line::lineSegmentTouchesOrCrossesLine(Line a, Line b){
    return (isPointOnLine(a,b.firstPoint) || isPointOnLine(a,b.secondPoint)) || xOrOperator(isPointRightOfLine(a,b.firstPoint),isPointRightOfLine(a,b.secondPoint)); 
}

bool Line::isPointOnLine(Line a, geometry_msgs::Point b){
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    p1.x = 0;
    p1.y = 0;

    p2.x = a.secondPoint.x - a.firstPoint.x;
    p2.y = a.secondPoint.y - a.firstPoint.y;

    Line aTmp(p1,p2);
    geometry_msgs::Point tmpPoint;
    tmpPoint.x = b.x - a.firstPoint.x;
    tmpPoint.y = b.y - a.firstPoint.y;

    double r = crossProduct(aTmp.secondPoint,tmpPoint);
    return fabs(r) < EPSILON;

}

bool Line::isPointRightOfLine(Line a, geometry_msgs::Point b){
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    p1.x = 0;
    p1.y = 0;

    p2.x = a.secondPoint.x-a.firstPoint.x;
    p2.y = a.secondPoint.y - a.firstPoint.y;
    Line aTmp(p1,p2);

    geometry_msgs::Point tmpPoint;
    tmpPoint.x = b.x - a.firstPoint.x;
    tmpPoint.y = b.y - a.firstPoint.y;
    return crossProduct(aTmp.secondPoint,tmpPoint) < 0;
}


double Line::crossProduct(geometry_msgs::Point a, geometry_msgs::Point b){
    return a.x*b.y - b.x*a.y;
}

bool Line::xOrOperator(bool a, bool b){
    return a ? !b : b;
}

bool Line::compareDoubles(double a, double b){
	double epsilon = 0.0001;    //Checks up to 4 digits after ,
	return fabs(a - b) < epsilon;
}
