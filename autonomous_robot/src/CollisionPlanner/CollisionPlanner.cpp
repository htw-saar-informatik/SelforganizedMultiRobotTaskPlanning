#include "CollisionPlanner.h"

CollisionPlanner::CollisionPlanner() {
	// TODO Auto-generated constructor stub

}

CollisionPlanner::~CollisionPlanner() {
	// TODO Auto-generated destructor stub
}


CollisionResult CollisionPlanner::calculateCollisions(std::vector<geometry_msgs::Point> pointsPathOne,std::vector<geometry_msgs::Point> pointsPathTwo, int radiusRobotOne, int radiusRobotTwo ){
    CollisionResult result;
    result.isValid = true;
    result.canDrive = true;

    //Check if enough points are available
    if( pointsPathOne.size() < 2){
        return result;
    }
    
    //Set up the robots
    Robot r1(radiusRobotOne,pointsPathOne);
    std::cout << "Setup Robot one" << std::endl;
    r1.setupRobot();

     //Draw Lines for this robot
    std::vector<geometry_msgs::Point> pointsVec1;
    std::vector<geometry_msgs::Point> pointsVec2;
    for(int i = 0; i < r1.smoothedTraceLeft.size();i++){
        pointsVec1.push_back(r1.smoothedTraceLeft[i].firstPoint);
        pointsVec1.push_back(r1.smoothedTraceLeft[i].secondPoint);
        pointsVec2.push_back(r1.smoothedTraceRight[i].firstPoint);
        pointsVec2.push_back(r1.smoothedTraceRight[i].secondPoint);
        
    }
    SharedFunctions::drawLines(pointsVec1,SharedFunctions::frame_id*100+3,true);
    SharedFunctions::drawLines(pointsVec2,SharedFunctions::frame_id*100+4,true);
    // sleep(1);
    SharedFunctions::drawLines(pointsVec1,SharedFunctions::frame_id*100+3,true);
    SharedFunctions::drawLines(pointsVec2,SharedFunctions::frame_id*100+4,true);

    //If robotTwo has only one point, he is not driving, he stands on a fixed position
    if( pointsPathTwo.size() == 1){
        result.canDrive = !checkIfPointIsOnWay(r1.segments,pointsPathTwo[0],radiusRobotTwo);
        return result;
    }

   
    
    // r1.printData();
    Robot r2(radiusRobotTwo,pointsPathTwo);
     std::cout << "Setup Robot two" << std::endl;
    r2.setupRobot();
    // r2.printData();
    
    //Because the start/end of a trace of the robots are open, check here first for start/endpoint of robotTwo who is driving, if it would intersect with the start/endPoint of robotOne
    //If so, robot one can not drive and has to wait

    if( checkIfPointIsOnWay(r1.segments,pointsPathTwo[pointsPathTwo.size()-1],radiusRobotTwo)){
        std::cout << "endPoint of other robot is on my way, i cannot drive" << std::endl;
        result.canDrive = false;
        return result;
    }


    std::vector<CollisionPath> collisionsRobotOne = getCollisionPaths(r1,r2);
    //std::vector<CollisionPath> collisionsRobotTwo = getCollisionPaths(r2,r1);

    // if ( collisionsRobotOne.size() != collisionsRobotTwo.size() ){
    //     result.isValid = false;
    //     ROS_ERROR("[CollisionPlanner::calculateCollisions] collisionPaht for both robots has different size");
    //     return result;
    // }

    std::vector<Collision> collisions;

    Segment segmentStart;
    // Segment segmentEnd;
    bool collisionPathMissing = false;
    

    //Find the collisionpositions, where robot1 hast to wait until robot2 reaches the point
    for ( int i = 0; i < collisionsRobotOne.size();i++){
        CollisionPath path = collisionsRobotOne[i];
        // FoundPath foundPath = findCollisionPath(path,collisionsRobotTwo);
        
        int id1 = path.startPoint.firstIntersectionSegmentId;
        int id2 = path.startPoint.secondIntersectionSegmentId;
        int id3 = path.endPoint.firstIntersectionSegmentId;
        int id4 = path.endPoint.secondIntersectionSegmentId;
        int tmpID;
        if (id1 > id2){
            tmpID = id1;
        }else{
            tmpID = id2;
        }
        if( tmpID < id3){
            tmpID = id3;
        }
        if( tmpID < id4){
            tmpID = id4;
        }
        Segment foundSegment= r2.segments[tmpID];

       
            // CollisionPath otherPath = foundPath.path;

            //Its possible, that if the both collisionsPaths are near together, the waitpoint for the robot1 would still collide with the path of robot2
            
            //Check robot r1, check if the robot at this waitposition (Draw a box around him) collides with the line between his leftintersection/rightintersection point
            id1 = path.startPoint.firstIntersection.getIntersection().segmentID;
            id2 = path.startPoint.secondIntersection.getIntersection().segmentID;
            if( id1 <= id2 ){
                segmentStart = r1.segments[id1];
            }else{
                segmentStart = r1.segments[id2];
            }
            Line line(path.startPoint.firstIntersection.getIntersection().intersectionPoint,path.startPoint.secondIntersection.getIntersection().intersectionPoint);
            Rectangle rec;
            bool doIntersect = false;

            do{
                rec.location.x = segmentStart.originalPointStart.x-radiusRobotOne;
                rec.location.y = segmentStart.originalPointStart.y + radiusRobotOne;
                rec.width = radiusRobotOne*2;
                rec.height = radiusRobotOne*2;
                if ( checkIfRectangleAndLineIntersect(rec,line)){
                    doIntersect = true;
                    if (segmentStart.id == 0){
                        ROS_ERROR("[CollisionPlanner::calculateCollisions] RobotOne has to go before first segment, should not be possible!");
                        result.isValid = false;
                        return result;
                    }else{
                        //Go back one segment
                        segmentStart = r1.segments[segmentStart.id -1];
                    }
                }else{
                    doIntersect = false;
                }

            }while(doIntersect);

            //Check robot2, if it intersects at his informPoint, move the position one segment backwards
            Line line2a(path.startPoint.firstIntersection.getIntersection().intersectionPoint, path.endPoint.secondIntersection.getIntersection().intersectionPoint);
            Line line2b(path.startPoint.secondIntersection.getIntersection().intersectionPoint,path.endPoint.firstIntersection.getIntersection().intersectionPoint);
            Rectangle rec2;
            doIntersect = false;

            do{
                rec2.location.x = foundSegment.originalPointEnd.x - radiusRobotTwo;
                rec2.location.y = foundSegment.originalPointEnd.y + radiusRobotTwo;
                rec2.width = radiusRobotTwo*2;
                rec2.height = radiusRobotTwo*2;

                if( checkIfRectangleAndLineIntersect(rec2,line2a) || checkIfRectangleAndLineIntersect(rec2,line2b)){
                    doIntersect = true;
                    if( foundSegment.id == r2.segments.size()-1){
                        ROS_INFO("[CollisionPlanner::calculateCollisions] Not possible to drive, Robot2 blocks the way at his end");
                        result.canDrive = false;
                        return result;
                    }else{
                        //Move position one segment backwards;
                        foundSegment = r2.segments[foundSegment.id+1];
                    }
                }else{
                    doIntersect = false;
                }
            }while(doIntersect);
        
            //Generate collision
            Collision c;
            c.waitPoint = segmentStart.originalPointStart;
            c.reachPoint = foundSegment.originalPointEnd;
            collisions.push_back(c);  
    }

    if ( collisions.size() == 0){

         if( checkIfPointIsOnWay(r2.segments,pointsPathOne[pointsPathOne.size()-1],radiusRobotOne)){
            std::cout << "The robot would collide on his endpoint, don't drive" << std::endl;
            result.canDrive = false;
            return result;
        }else  if( checkIfPointIsOnWay(r1.segments,pointsPathTwo[0],radiusRobotOne)){
            result.canDrive = false;
            return result;
        }else{
            return result;
        }
    }

    //Its possible for the collisions that they do overlap

    //Check overlapping for startPoint
    std::vector<Collision> collisions1;
    Collision lastCollision;
    bool lastCollisionSet = false;
    for( int i = 0; i < collisions.size(); i++){
        Collision currentCollision = collisions[i];

        if ( !lastCollisionSet ){
            lastCollision = currentCollision;
            lastCollisionSet = true;
        }else{
            
            if( compareDoubles(lastCollision.waitPoint.x,currentCollision.waitPoint.x) && compareDoubles(lastCollision.waitPoint.y,currentCollision.waitPoint.y)){

                lastCollision = currentCollision;
            }else{
                collisions1.push_back(lastCollision);
                lastCollision = currentCollision;
            }
        }
    }
    //Last element is missing
    collisions1.push_back(collisions[collisions.size()-1]);

    //Check overlapping for Endpoint
    std::vector<Collision> collisions2;
    lastCollisionSet = false;
    for( int i = 0; i < collisions1.size();i++){
        Collision currentCollision = collisions1[i];
        if ( !lastCollisionSet ){
            lastCollision = currentCollision;
            lastCollisionSet = true;
        }else{
            if ( compareDoubles(lastCollision.reachPoint.x,currentCollision.reachPoint.x) && compareDoubles(lastCollision.reachPoint.y, currentCollision.reachPoint.y)){
                //Do nothing, hold first one found
            }else{
                collisions2.push_back(lastCollision);
                lastCollision = currentCollision;
            }
        }
    }
    //last element ist missing
    collisions2.push_back(lastCollision);
   
    result.isValid = true;
    result.canDrive = true;
    result.collisions = collisions2;

    return result;
}

/**
 * Returns all CollisionPaths on the path for Robot1 for the contur of Robot2
 */ 
std::vector<CollisionPath>  CollisionPlanner::getCollisionPaths(Robot robotOne, Robot robotTwo){
    std::vector<LineElement> lineElements;
    std::vector<CollisionPath> collisionPaths;
    std::vector<Segment> segmentList = robotOne.segments; 

    std::vector<Line> contur = robotTwo.getContur();

    if ( robotOne.pointsPath.size() < 2 || robotTwo.pointsPath.size() < 2){
        std::cout << "Not enough points on trace " << std::endl;
        return collisionPaths;
    }
    
    calculateIntersectionsOnContur(robotOne,contur,robotTwo.smoothedTraceLeft.size());
    // Segment* s1 = map[0][0].segmentID;
    // Segment* s2 = map[1][0].segmentID;
    //Generate LineElements, each intersection gets its own lineElement. If a line has no intersection, generate a lineElement with no intersection
    //This is done to go through the list of intersections and search for them, thats easier than going through the vectors of intersections for each line
    bool collisionFound = false;
    for ( int i = 0; i < contur.size(); i++){
        //Search for the element
        std::map< int,std::vector<Intersection> >::iterator it = map.find(i);
		if( it != map.end()){
            //Element found, so there are intersections for this line
            std::vector<Intersection> intersections = it->second;
            collisionFound = true;
            //For every intersection create his own LineElement
    		for( int j = 0; j < intersections.size(); j++){
                LineElement e(contur[i]);
                e.intersections.push_back(intersections[j]);
                lineElements.push_back(e);
			}
		}else{
            //Add LineElement without intersection
            LineElement e(contur[i]);
            lineElements.push_back(e);
		}
    }

    //Check if there are intersections or not
    if ( !collisionFound ){
        return collisionPaths;
    }

    //Shift LineElements until the list starts with an Element who has an intersection
    while ( !lineElements[0].intersections.size() > 0 ){
        LineElement tmpLine = lineElements[0];
        lineElements.erase(lineElements.begin());   //delete first element
        lineElements.push_back(tmpLine);            //add element at the end
    }

    for(int i = 0; i < lineElements.size();i++){
        LineElement e = lineElements[i];
        if( e.intersections.size() > 0){
            Intersection::EType type = e.getIntersection().type;
        if( type == Intersection::EType::aMinus ){
            std::cout << "aMinus" << std::endl;
        }else if ( type == Intersection::EType::aPlus){
            std::cout << "aPlus" << std::endl;
        }else if ( type == Intersection::EType::bMinus){
            std::cout << "bMinus" << std::endl;
        }else if ( type == Intersection::EType::bPlus){
            std::cout << "bPlus" << std::endl;
        }
        }
        
    }

    //Add first element additionaly at the end, so that it not only checks from this element also to this element
    lineElements.push_back(lineElements[0]);

    collisionPaths = calculateSubPaths(lineElements,segmentList);
    map.clear();
    return collisionPaths;
}

/**
 * Calculates all intersections between the path of robotOne and the Contur of robotTwo. Intersections are stored in a map
 * with index = nr of line in contur and as value a vector of intersections in a distance based order
 * 
 */ 
void CollisionPlanner::calculateIntersectionsOnContur(Robot robotOne, std::vector<Line> contur,int countSegmentsRobotTwo){
    Line lineLeft;
    Line lineRight;
    int counter = 0;
    // std::vector<Segment> segmentList = robotOne.segments;
    // std::vector<Line> contur = robotTwo.getContur();

    for ( int i = 0; i < robotOne.smoothedTraceLeft.size(); i++){
        lineLeft = robotOne.smoothedTraceLeft[i];
        lineRight = robotOne.smoothedTraceRight[i];
        counter = 0;
       
    
        //Go through entire contur of robot2 and look if one of the lines from the contur intersect with line left or right
        for( int j = 0; j < contur.size();j++){
            int indexSegment;
            Line currentLine = contur[j];
            double orientation;
            Intersection intersectionLeft;
            Intersection intersectionRight;

            if( counter <= (countSegmentsRobotTwo-1) ){
                indexSegment = counter;
            }else{
                indexSegment = countSegmentsRobotTwo - (counter - countSegmentsRobotTwo +1);
            }

            // if((currentLine.firstPoint.x == currentLine.secondPoint.x) && (currentLine.firstPoint.y == currentLine.secondPoint.y) ){
			// 	//Empty line, do not process
			// }else{
                //Check left side for intersection
                orientation = getOrientation(lineLeft,currentLine.firstPoint);
                if ( Line::doLinesIntersect(lineLeft,currentLine) ){
                    geometry_msgs::Point intersectionPoint = Line::getIntersection(lineLeft,currentLine).firstPoint;

                    if ( orientation > 0 ){
                        intersectionLeft = Intersection(intersectionPoint,lineLeft,currentLine,Intersection::EType::bPlus,i,indexSegment);
                    }else if ( orientation < 0 ){
                        intersectionLeft = Intersection(intersectionPoint,lineLeft,currentLine,Intersection::EType::bMinus,i,indexSegment);
                    }else{
                        //When 0, the point is exactly on line, look at endpoint
                        orientation = getOrientation(lineLeft,currentLine.secondPoint);

                        if ( orientation > 0 ){
                            intersectionLeft = Intersection(intersectionPoint,lineLeft,currentLine,Intersection::EType::bMinus,i,indexSegment);
                        }else if ( orientation < 0 ){
                            intersectionLeft = Intersection(intersectionPoint,lineLeft,currentLine,Intersection::EType::bPlus,i,indexSegment);
                        }else{
                            //Parallel
                            ROS_INFO("[CollisionPlanner::getCollisionPaths] Lines are parallel!");
                            SharedFunctions::countLinesAreParallel++;
                        }
                    }
                    //Save this intersection
                    addToMap(counter,intersectionLeft);
                    
                }

                //Check right side for intersection
                orientation = getOrientation(lineRight,currentLine.firstPoint);
                if ( Line::doLinesIntersect(lineRight,currentLine) ){
                    geometry_msgs::Point intersectionPoint = Line::getIntersection(lineRight,currentLine).firstPoint;

                    if ( orientation > 0 ){
                        intersectionRight = Intersection(intersectionPoint,lineRight,currentLine,Intersection::EType::aMinus,i,indexSegment);
                    }else if ( orientation < 0 ){
                        intersectionRight = Intersection(intersectionPoint,lineRight,currentLine,Intersection::EType::aPlus,i,indexSegment);
                    }else{
                        //When 0, the point is exactly on line, look at endpoint
                        orientation = getOrientation(lineRight,currentLine.secondPoint);

                        if ( orientation > 0 ){
                            intersectionRight = Intersection(intersectionPoint,lineRight,currentLine,Intersection::EType::aPlus,i,indexSegment);
                        }else if ( orientation < 0 ){
                            intersectionRight = Intersection(intersectionPoint,lineRight,currentLine,Intersection::EType::aMinus,i,indexSegment);
                        }else{
                            //Parallel
                            ROS_INFO("[CollisionPlanner::getCollisionPaths] Lines are parallel 2!");
                            SharedFunctions::countLinesAreParallel++;
                        }
                    }
                    //Save this intersection
                    addToMap(counter,intersectionRight);
                }
                counter++;
            // }
        }
    }
}

/**
 *  iterates ofer all LineElements and searchs for subpaths.
 * Returns a list of segments, each segment hast a list wiht subPathPoints, correctly orderer in there way along the path 
 */
std::vector<CollisionPath> CollisionPlanner::calculateSubPaths(std::vector<LineElement> lineElements,std::vector<Segment> segmentList){
    std::vector<CollisionPath> collisionPaths;

    LineElement lastElement;
    bool lastElementSet = false;
    for( int i = 0; i < lineElements.size(); i++){
        LineElement element = lineElements[i];
        
        if( !lastElementSet){
            lastElement = element;
            lastElementSet = true;
        }else{

            if ( element.intersections.size() != 0){

                if( lastElement.getIntersection().type == Intersection::EType::aPlus && element.getIntersection().type == Intersection::EType::aMinus ){
                    //If aPlus is left aMinus on the path for the Robot, then this is a complete collisionPath
                    double distance1 = getDistanceOnPathforIntersection(lastElement.getIntersection(),segmentList);
                    double distance2 = getDistanceOnPathforIntersection(element.getIntersection(),segmentList);
                    
                    if( distance1 < distance2 ){
                        int idSegmentBegin = lastElement.getIntersection().segmentID;
                        int idSegmentEnd = element.getIntersection().segmentID;

                        segmentList[idSegmentBegin].addSubPathPoint( SubPathPoint(lastElement.getIntersection().intersectionPoint,lastElement,element,SubPathPoint::SubPathType::beginSubPath,lastElement.getIntersection().segmentIDRobotTwo,element.getIntersection().segmentIDRobotTwo));
                        segmentList[idSegmentEnd].addSubPathPoint( SubPathPoint(element.getIntersection().intersectionPoint,lastElement,element,SubPathPoint::SubPathType::endSubPath,lastElement.getIntersection().segmentIDRobotTwo,element.getIntersection().segmentIDRobotTwo) );
                    }
                }else if ( lastElement.getIntersection().type == Intersection::EType::aPlus && element.getIntersection().type == Intersection::EType::bMinus){
                    //Begin of a collision subpath
                    int idSegment;
                    geometry_msgs::Point p;
                    
                    //Choose the segment with lowest id
                    if( lastElement.getIntersection().segmentID <= element.getIntersection().segmentID ){
                        idSegment = lastElement.getIntersection().segmentID;
                        p = lastElement.getIntersection().intersectionPoint;
                    }else{
                        idSegment = element.getIntersection().segmentID;
                        p = element.getIntersection().intersectionPoint;
                    }
                    segmentList[idSegment].addSubPathPoint( SubPathPoint(p,lastElement,element,SubPathPoint::SubPathType::beginSubPath,lastElement.getIntersection().segmentIDRobotTwo,element.getIntersection().segmentIDRobotTwo) );
                }else if ( lastElement.getIntersection().type == Intersection::EType::bPlus && element.getIntersection().type == Intersection::EType::bMinus){
                    //If b- is left b+ on the path of the robot, then this is a complete collisionPath
                    double distance1 = getDistanceOnPathforIntersection(lastElement.getIntersection(),segmentList);
                    double distance2 = getDistanceOnPathforIntersection(element.getIntersection(), segmentList);

                    // if( Line::doLinesIntersect(lastElement.getIntersection().secondLine,lastElement.getIntersection().firstLine) ){
                    //     Line l = Line::getIntersection(lastElement.getIntersection().firstLine,lastElement.getIntersection().secondLine);
                    //     std::cout << std::endl;
                    // }

                    if ( distance1 > distance2 ){
                        int idSegmentBegin = element.getIntersection().segmentID;
                        int idSegmentEnd = lastElement.getIntersection().segmentID;

                        segmentList[idSegmentBegin].addSubPathPoint( SubPathPoint(element.getIntersection().intersectionPoint,lastElement,element,SubPathPoint::SubPathType::beginSubPath,lastElement.getIntersection().segmentIDRobotTwo,element.getIntersection().segmentIDRobotTwo) );
                        segmentList[idSegmentEnd].addSubPathPoint( SubPathPoint(lastElement.getIntersection().intersectionPoint,lastElement,element,SubPathPoint::SubPathType::endSubPath,lastElement.getIntersection().segmentIDRobotTwo,element.getIntersection().segmentIDRobotTwo) );
                    }
                }else if ( lastElement.getIntersection().type == Intersection::EType::bPlus && element.getIntersection().type == Intersection::EType::aMinus){
                    //End collsion Subpath
                    int idSegment;
                    geometry_msgs::Point p;
                    
                    //Choose the segment with highest id
                    if ( lastElement.getIntersection().segmentID >= element.getIntersection().segmentID ){
                        idSegment = lastElement.getIntersection().segmentID;
                        p = lastElement.getIntersection().intersectionPoint;
                    }else{
                        idSegment = element.getIntersection().segmentID;
                        p = element.getIntersection().intersectionPoint;
                    }

                    segmentList[idSegment].addSubPathPoint( SubPathPoint(p,lastElement,element,SubPathPoint::SubPathType::endSubPath,lastElement.getIntersection().segmentIDRobotTwo,element.getIntersection().segmentIDRobotTwo) );
                }
                lastElement = element;
            }
        }
    }

    //Create a list of all subPath points from all Segments. Is easier to check
    std::vector<SubPathPoint> subPahtPoints;
    for( int i = 0; i < segmentList.size();i++){
        if( segmentList[i].subPahtPoints.size() > 0){
            //Append all points to the list
            subPahtPoints.insert(subPahtPoints.end(), segmentList[i].subPahtPoints.begin(), segmentList[i].subPahtPoints.end());
        }
    }

    //Check if there are subPaths
    if( subPahtPoints.size() == 0){
        ROS_INFO("[CollisionPlanner::calculateSubPaths] no subPathPoints found");
        return collisionPaths;
    }

    if( subPahtPoints[0].type == SubPathPoint::SubPathType::endSubPath){
        ROS_ERROR("[CollisionPlanner::calculateSubPaths] Collision path starts with an closed way!");
        SharedFunctions::countCollisionPathStartsWithClosedWay++;
    }

    //Its possible for the paths to overlap
    //Search for the cohesive start and end points for a collisionpath
    SubPathPoint startPoint;
    int countElements = 0;
    bool startPointSet = false;
    for ( int i = 0; i < subPahtPoints.size(); i++){
        SubPathPoint point = subPahtPoints[i];

        if( point.type == SubPathPoint::SubPathType::beginSubPath){
            countElements+=1;
            if ( !startPointSet ){
                startPoint = point;
                startPointSet = true;
            }
        }else if ( point.type = SubPathPoint::SubPathType::endSubPath ){
            countElements-=1;
            if ( countElements == 0 ){
                collisionPaths.push_back( CollisionPath(startPoint,point));
                startPointSet = false;
            }
        }
    }

    if ( countElements != 0 ){
        //Its possible that the path for the robot end in the path of r2, therefore no endsubpath is found for the last startSubPath. Check this and add it manually
        if( countElements == 1){
            SubPathPoint p = subPahtPoints[subPahtPoints.size()-1];
            if( p.type == SubPathPoint::SubPathType::beginSubPath){
                SubPathPoint p2(p.p,p.firstIntersection,p.secondIntersection,SubPathPoint::SubPathType::endSubPath,p.firstIntersectionSegmentId,p.secondIntersectionSegmentId);
                collisionPaths.push_back(CollisionPath(p,p2));
                ROS_INFO("[CollisionPlanner::calculateSubPaths] endSubPath missing, add it manually");
            }
        }else{
            ROS_ERROR("[CollisionPlanner::calculateSubPaths] Not every path is closed");
            SharedFunctions::notEveryPathIsClosed++;
        }

    }
    return collisionPaths;

}

/**
 * Gets the distance from first segment to the segment where the intersection occurs and to the actual position in this segment
 */ 
double CollisionPlanner::getDistanceOnPathforIntersection(Intersection intersection,  std::vector<Segment> segmentList){
    double distance;
    int counter = 0;
    while ( counter != intersection.segmentID ){
        distance += segmentList[counter].getSegmentLenght();
        counter+=1;
    }
    //Add distance to point in segment
    distance += segmentList[intersection.segmentID].getDistanceToPoint(intersection.intersectionPoint);
    return distance;
}

/**
 * Adds an Intersection to a Map. index = index of Line on Contur to add the intersection
 * Intersection gets added in a way that the intersections for the line are orderd, e.g the intersections
 * are orderd after distance to the startPoint of the line
 */ 
void CollisionPlanner::addToMap(int index, Intersection intersection){
    std::vector<Intersection> list;
    std::vector<Intersection> tmpList = map[index];;
    tmpList.push_back(intersection);

    int counter = 0;
    double distance = 0;
    Intersection element;
    Intersection currentElement;
    geometry_msgs::Point firstPoint;

    //Search for intersection with lowest distance and add it to list
    while ( tmpList.size() > 0){
        element = tmpList[0];
        firstPoint = element.secondLine.firstPoint;
        distance = getDistanceBetweenPoints(firstPoint,element.intersectionPoint);
        counter = 0;

        for( int i = 0; i < tmpList.size(); i++){
            currentElement = tmpList[i];
            firstPoint = currentElement.secondLine.firstPoint;

            double dist = getDistanceBetweenPoints(firstPoint,currentElement.intersectionPoint);
            if ( dist < distance ){
                element = currentElement;
                distance = getDistanceBetweenPoints(firstPoint,currentElement.intersectionPoint);
                counter = i;
            }
        }
        list.push_back(element);
        tmpList.erase(tmpList.begin()+counter);
    }
    map[index] = list;
}

double CollisionPlanner::getDistanceBetweenPoints(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return sqrt( pow(p2.x-p1.x,2) + pow(p2.y-p1.y,2) );
}
double CollisionPlanner::getOrientation(Line line, geometry_msgs::Point point){
    return ( line.secondPoint.x - line.firstPoint.x) * (point.y - line.firstPoint.y) - (line.secondPoint.y - line.firstPoint.y) * (point.x - line.firstPoint.x);
}


bool CollisionPlanner::checkIfPointIsOnWay(std::vector<Segment> segments, geometry_msgs::Point p,double radius){
    Rectangle r;
    r.location.x = p.x-radius;
    r.location.y = p.y + radius;
    r.height = radius*2;
    r.width = radius*2;

    for( int i = 0; i < segments.size(); i++){
        Segment s = segments[i];
        Line l1 = s.shortLineLeft;
        Line l2(s.shortLineLeft.secondPoint,s.shortLineRight.secondPoint);
        Line l3 = s.shortLineRight;
        Line l4(s.shortLineLeft.firstPoint,s.shortLineRight.firstPoint);

        if( checkIfRectangleAndLineIntersect(r,l1) || checkIfRectangleAndLineIntersect(r,l2) || checkIfRectangleAndLineIntersect(r,l3) || checkIfRectangleAndLineIntersect(r,l4)){
            return true;
        }

        // if( segments[i].isPointInSegment(p)){
        //     return true;
        // }
    }
    return false;
}

 bool CollisionPlanner::containsPoint(std::vector<geometry_msgs::Point> points, geometry_msgs::Point point){
     bool contains = false;
     geometry_msgs::Point currentPoint;
     for( int i = 0; i < points.size();i++){
        currentPoint = points[i];

        if ( compareDoubles(currentPoint.x,point.x) && compareDoubles(currentPoint.y,point.y)){
            contains = true;
        }

        // if ( currentPoint.x == point.x && currentPoint.y == point.y){
        //     contains = true;
        // }   
     }
     return contains;
 }

bool CollisionPlanner::checkIfRectangleAndLineIntersect(Rectangle& rect, Line& line){
    bool intersect = false;
    geometry_msgs::Point ul,ur,dl,dr;    //Upper left;
    ul = rect.location;
    
    ur.x = rect.location.x + rect.width;
    ur.y = rect.location.y;

    dr.x = rect.location.x + rect.width;
    dr.y = rect.location.y - rect.height;

    dl.x = rect.location.x;
    dl.y = rect.location.y - rect.height;


    Line line1(ul,ur);
    Line line2(ur,dr);
    Line line3(dl,dr);
    Line line4(ul,dl);

    if ( Line::doLinesIntersect(line1,line) || Line::doLinesIntersect(line2,line) || Line::doLinesIntersect(line3,line) || Line::doLinesIntersect(line4,line)){
        intersect = true;
    }

    //Check if line is in the rectangle
    geometry_msgs::Point pt;
    pt.x = line.firstPoint.x;
    pt.y = line.firstPoint.y;

    if( (pt.x >= rect.location.x) && ( pt.x <= (rect.location.x + rect.width)) ){
        if ( ( pt.y <= rect.location.y ) && ( pt.y >= (rect.location.y-rect.height) ) ){
            intersect = true;
        }
    }

    pt.x = line.secondPoint.x;
    pt.y = line.secondPoint.y;
    if( (pt.x >= rect.location.x) && ( pt.x <= (rect.location.x + rect.width)) ){
        if ( ( pt.y <= rect.location.y ) && ( pt.y >= (rect.location.y-rect.height) ) ){
            intersect = true;
        }
    }
    return intersect;
}

bool CollisionPlanner::compareDoubles(double a, double b){
	double epsilon = 0.0001;    //Checks up to 4 digits after ,
	return fabs(a - b) < epsilon;
}