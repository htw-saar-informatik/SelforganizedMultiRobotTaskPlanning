#ifndef LINEELEMENT_H_
#define LINEELEMENT_H_

#include "Line.h"
#include "Intersection.h"
#include <vector>

/**
 * Stores all intersections on a line. Multiple intersections on a line are possible
 */ 
class LineElement {
public:
	LineElement();
    LineElement(Line l);
	virtual ~LineElement();

    Line line;
    std::vector<Intersection> intersections;

    Intersection getIntersection();
};

#endif /* LINEELEMENT_H_ */
