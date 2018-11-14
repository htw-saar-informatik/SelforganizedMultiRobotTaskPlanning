
#include "Robot.h"
Robot::Robot(int radius, std::vector<geometry_msgs::Point> pointsPath) {
	this->radius = radius;
	this->pointsPath = pointsPath;

}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}


void Robot::setupRobot(){
	geometry_msgs::Point currentPoint;
	geometry_msgs::Point lastPoint;
	bool lastPointSet = false;

	std::vector<Line> traceLeft;	//Left side of trace for this robot
	std::vector<Line> traceRight;
	std::vector<Line> result;
	std::cout << "Size: " << pointsPath.size() << std:: endl;
	//Generate left and right trace between all following points in path
	for ( int i = 0; i < (pointsPath.size());i++){
		currentPoint = pointsPath[i];
		if ( lastPointSet ){
			result = calculateTraceForPoints(lastPoint,currentPoint);
			traceRight.push_back(result[0]);
			traceLeft.push_back(result[1]);
		}

		lastPoint = currentPoint;
		lastPointSet = true;
	}

	//Save original lines before smoothing
	originalTraceLeft = traceLeft;
	originalTraceRight = traceRight;
	
	std::cout << "Smooth left" << std::endl;
	smoothLines(traceLeft);
	std::cout << "Smooth right" << std::endl;
	smoothLines(traceRight);
	smoothedTraceLeft = traceLeft;
	smoothedTraceRight = traceRight;

	//Generate segments
	for( int i = 0; i < originalTraceLeft.size();i++){
		Segment s(originalTraceLeft[i],originalTraceRight[i],smoothedTraceLeft[i],smoothedTraceRight[i],i,pointsPath[i],pointsPath[i+1]);
		segments.push_back(s);
	}
}

void Robot::printData(){
	std::cout << "OriginalLinesLeft" << std::endl;
	for( int i = 0; i < originalTraceLeft.size();i++){
		std::cout << i << ") F [" << originalTraceLeft[i].firstPoint.x << " " << originalTraceLeft[i].firstPoint.y << "]";
		std::cout << "S [" << originalTraceLeft[i].secondPoint.x << " " << originalTraceLeft[i].secondPoint.y << "]" << std::endl;;
	}

	std::cout << "SmoothedTraceLeft" << std::endl;
	for( int i = 0; i < smoothedTraceLeft.size();i++){
		std::cout << i << ") F [" << smoothedTraceLeft[i].firstPoint.x << " " << smoothedTraceLeft[i].firstPoint.y << "]";
		std::cout << "S [" << smoothedTraceLeft[i].secondPoint.x << " " << smoothedTraceLeft[i].secondPoint.y << "]" << std::endl;;
	}

	std::cout << "OriginalLinesRight" << std::endl;
	for( int i = 0; i < originalTraceRight.size();i++){
		std::cout << i << ") F [" << originalTraceRight[i].firstPoint.x << " " << originalTraceRight[i].firstPoint.y << "]";
		std::cout << "S [" << originalTraceRight[i].secondPoint.x << " " << originalTraceRight[i].secondPoint.y << "]" << std::endl;;
	}

	std::cout << "SmoothedTraceRight" << std::endl;
	for( int i = 0; i < smoothedTraceRight.size();i++){
		std::cout << i << ") F [" << smoothedTraceRight[i].firstPoint.x << " " << smoothedTraceRight[i].firstPoint.y << "]";
		std::cout << "S [" << smoothedTraceRight[i].secondPoint.x << " " << smoothedTraceRight[i].secondPoint.y << "]" << std::endl;;
	}
}


/**
 *  Calculates the right and the left line between points p1 and p2
 */ 
std::vector<Line> Robot::calculateTraceForPoints(geometry_msgs::Point p1, geometry_msgs::Point p2){
	std::vector<Line> result;

	//If both x coordinates are equal, the linearfunction can not be calculated
	if ( p1.x == p2.x ){

		//Calculate Points for p1
		geometry_msgs::Point p11;
		geometry_msgs::Point p12;

		if ( p1.y < p2.y ){
			p11.x = p1.x+radius;
			p11.y = p1.y-radius;

			p12.x = p1.x-radius;
			p12.y = p1.y-radius;
		}else{
			p11.x = p1.x-radius;
			p11.y = p1.y+radius;

			p12.x = p1.x+radius;
			p12.y = p1.y+radius;
		}

		//Calculate Points for p2
		geometry_msgs::Point p21;
		geometry_msgs::Point p22;

		if ( p2.y < p1.y ){
			p21.x = p2.x-radius;
			p21.y = p2.y-radius;

			p22.x = p2.x+radius;
			p22.y = p2.y-radius;
		}else{
			p21.x = p2.x+radius;
			p21.y = p2.y+radius;

			p22.x = p2.x-radius;
			p22.y = p2.y+radius;
		}

		Line lineRight(p11,p21);
		Line lineLeft(p12,p22);
		result.push_back(lineRight);
		result.push_back(lineLeft);

	}else if( p1.y == p2.y ){
		//same as above

		//Calculates points for p1
		geometry_msgs::Point p11;
		geometry_msgs::Point p12;

		if ( p1.x < p2.x ){
			p11.x = p1.x-radius;
			p11.y = p1.y-radius;

			p12.x = p1.x-radius;
			p12.y = p1.y+radius;
		}else{
			p11.x = p1.x+radius;
			p11.y = p1.y+radius;

			p12.x = p1.x+radius;
			p12.y = p1.y-radius;
		}

		//Calculate Points for p2
		geometry_msgs::Point p21;
		geometry_msgs::Point p22;

		if ( p2.x < p1.x ){
			p21.x = p2.x-radius;
			p21.y = p2.y+radius;

			p22.x = p2.x-radius;
			p22.y = p2.y-radius;
		}else{
			p21.x = p2.x+radius;
			p21.y = p2.y-radius;

			p22.x = p2.x+radius;
			p22.y = p2.y+radius;
		}

		Line lineRight(p11,p21);
		Line lineLeft(p12,p22);
		result.push_back(lineRight);
		result.push_back(lineLeft);
	}else{
		std::vector<geometry_msgs::Point> listPoints1 = calculatePointsOnCircle(p1,p2,true);
		std::vector<geometry_msgs::Point> listPoints2 = calculatePointsOnCircle(p2,p1,false);

		Line lineRight(listPoints1[0],listPoints2[0]);
		Line lineLeft(listPoints1[1],listPoints2[1]);

		result.push_back(lineRight);
		result.push_back(lineLeft);
	}
	return result;
}

/**
 * Calculates the points on the circle. p1 = Point for calculationg coordinates, p2 = Point where the line with p1 ends
 */ 
std::vector<geometry_msgs::Point> Robot::calculatePointsOnCircle(geometry_msgs::Point p1, geometry_msgs::Point p2, bool outgoingPoint){
	std::vector<geometry_msgs::Point> list;

	//1) Calculate function between p1 and p2
	LinearFunction y = calculateLinearFunction(p1,p2);
	geometry_msgs::Point p11;
	p11.x = p1.x;
	p11.y = p1.y;
	geometry_msgs::Point p12;
	p12.x = p1.x;
	p12.y = p1.y;

	//2) Move the points forwards/backwards to cover full circle, not only half circle

	//Calculate the angle between the linearfunction and the line between p1 and x-axis
	double angleXAxis = fabs( atan(y.m) * 180 / M_PI );
	//Calculate the angle between the vertical and the linearfunction with p1
	double angleBetweenFunctionAndVertical = 90-angleXAxis;

	double xLenght = sin( angleBetweenFunctionAndVertical*M_PI/180);
	double yLenght = cos( angleBetweenFunctionAndVertical*M_PI/180);
	
	//Check the orientation for the linearfunction
	if ( (p1.x < p2.x) && (p1.y < p2.y) ){
		p11.x = p11.x - (xLenght*radius);
		p11.y = p11.y - (yLenght*radius);
	}else if ( (p1.x > p2.x) && (p1.y > p2.y) ){
		p11.x = p11.x + (xLenght*radius);
		p11.y = p11.y + (yLenght*radius);
	}else if ( (p1.x < p2.x) && (p1.y > p2.y) ){
		p11.x = p11.x - (xLenght*radius);
		p11.y = p11.y + (yLenght*radius);
	}else{
		p11.x = p11.x + (xLenght*radius);
		p11.y = p11.y - (yLenght*radius);
	}

	//Set p2
	p12.x = p11.x;
	p12.y = p11.y;

	//3) move the points outwards on the circle
	//Calculate orthogonal function with the linearfunction between p1 and p2, through the point p11
	LinearFunction yb = calculateOrthogonalFunctionForPoint(y,p11);
	double angle = fabs( atan(yb.m) );
	//leghtAdjacent is the distance for a circle with radius 1, because angle is equal if x-achse would be higher
	double lenghtAdjacent = cos( angle );
	double offset = lenghtAdjacent*radius ;

	//Make sure that for the entire trace all left sides are on the left side and right side on the right
	if ( p1.x < p2.x ){
		if ( p1.y < p2.y ){
			p11.x = p11.x-offset;
			p11.y = yb.getFx(p11.x);

			p12.x = p12.x+offset;
			p12.y = yb.getFx(p12.x);
		}else{
			p11.x = p11.x+offset;
			p11.y = yb.getFx(p11.x);

			p12.x = p12.x-offset;
			p12.y = yb.getFx(p12.x);
		}
	}else{
		if ( p1.y < p2.y ){
			p11.x = p11.x-offset;
			p11.y = yb.getFx(p11.x);

			p12.x = p12.x+offset;
			p12.y = yb.getFx(p12.x);
		}else{
			p11.x = p11.x+offset;
			p11.y = yb.getFx(p11.x);

			p12.x = p12.x-offset;
			p12.y = yb.getFx(p12.x);
		}
	}

	if ( !outgoingPoint ){
		list.push_back(p11);
		list.push_back(p12);
	} else{
		list.push_back(p12);
		list.push_back(p11);
	}

	return list;
}

LinearFunction Robot::calculateOrthogonalFunctionForPoint(LinearFunction& lf, geometry_msgs::Point p){
	double m,b;

	m = (-1) / lf.m;
	b = p.y - (m*p.x);
	LinearFunction function(m,b);
	return function;
}

/**
 * Calculates a linearfunctin between two points
 */
LinearFunction Robot::calculateLinearFunction(geometry_msgs::Point p1, geometry_msgs::Point p2){
	double m;
	double b1;
	double b2;

	m = (p2.y - p1.y) / (p2.x-p1.x);
	//Point 1
	b1 = p1.y - (m * p1.x);
	//Point 2
	b2 = p2.y - (m * p2.x);

	//b1 and b2 should be equal

	LinearFunction y(m,b1);
	return y;
}

/**
 * Check every following lines if they intersect, if so make them smooth by cutting the lines at intersection Point
 */ 
void Robot::smoothLines(std::vector<Line>& lines){
	// std::vector<Line> newLines;
	for( int j = 0; j < (lines.size()-1);j++) {
		Line& currentLine = lines[j];
		Line& followingLine = lines[j+1];

		bool doIntersect = Line::doLinesIntersect(currentLine,followingLine);

		if( j == 6){
			std::cout << "d";
		}

		if( doIntersect ){
			Line l = Line::getIntersection(currentLine,followingLine);
			geometry_msgs::Point p;
			Line originalLineC = currentLine;
			Line originalLineF = followingLine;

			if( (l.firstPoint.x == l.secondPoint.x) && (l.firstPoint.y == l.secondPoint.y) ){
				//Single point intersection
				p = l.firstPoint;
			}else{
				//Line intersection, use middlepoint of the line
				p.x = (l.firstPoint.x + l.secondPoint.x)/2;
				p.y = (l.firstPoint.y + l.secondPoint.y)/2;
			}
			
			currentLine.secondPoint.x = p.x;
			currentLine.secondPoint.y = p.y;
			followingLine.firstPoint.x = p.x;
			followingLine.firstPoint.y = p.y;
			
			// if(currentLine.firstPoint.x == currentLine.secondPoint.x && currentLine.firstPoint.y == currentLine.secondPoint.y){
			// 	std::cout << "Gleich" << std::endl;
			// }
			// if(followingLine.firstPoint.x == followingLine.secondPoint.x && followingLine.firstPoint.y == followingLine.secondPoint.y){
			// 	std::cout << "Gleich" << std::endl;
			// }
		}
	}
}

/**
 * Returns the contur for the robot (The contur are the lines in the order they can get traversed)
 * Left side of lines stays unchanged, right side has to be visited revesed and every line hast to change her direction
 */ 
std::vector<Line> Robot::getContur(){
	std::vector<Line> contur;

	//Add left side unchanged
	// for ( int i = 0; i < smoothedTraceLeft.size(); i++){
	// 	contur.push_back(Line(smoothedTraceLeft[i].firstPoint,smoothedTraceLeft[i].secondPoint));
	// }
	contur = smoothedTraceLeft;

	//Right side has to be added in reversed order
	std::vector<Line> tmpRightSide = smoothedTraceRight;
	std::reverse(tmpRightSide.begin(),tmpRightSide.end());

	//Lines has to change the direction
	for( int i = 0; i < tmpRightSide.size(); i++){
		contur.push_back(Line(tmpRightSide[i].secondPoint,tmpRightSide[i].firstPoint));
	}

	return contur;
}

void Robot::drawLine(Line& l){
	std::cout << "Draw a line now" <<std::endl;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration();
	marker.id = 0;	//Has to be unique for every line
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	
	// marker.pose.position.x = 1;
	// marker.pose.position.y = 1;
	// marker.pose.position.z = 1;
	// marker.pose.orientation.x = 0.0;
	// marker.pose.orientation.y = 0.0;
	// marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	// marker.scale.y = 0.1;
	// marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	marker.points.push_back(l.firstPoint);
	marker.points.push_back(l.secondPoint);

	SharedFunctions::visPub->publish( marker );
}

//Sets a marker in rviz
void Robot::setMarker(geometry_msgs::Point p){
	std::cout << "SetMarker" << std::endl;

    static ros::Publisher vis_pub = SharedFunctions::nodeHandler->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0.0;
    

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    //Größe der Kugel
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();

	vis_pub.publish( marker );
    
}

void Robot::sleep(int millis){
     std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}