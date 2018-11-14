#include "LineElement.h"

LineElement::LineElement() {
	// TODO Auto-generated constructor stub

}

LineElement::LineElement(Line l) {
	this->line = l;
}
LineElement::~LineElement() {
	// TODO Auto-generated destructor stub
}

/**
 * Returns the first intersection
 */ 
Intersection LineElement::getIntersection(){
    if ( intersections.size() != 0){
        return intersections[0];
    }
}

