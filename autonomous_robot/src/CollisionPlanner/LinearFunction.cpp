#include "LinearFunction.h"

LinearFunction::LinearFunction(double m, double b) {
	this->m = m;
    this->b = b;

}

LinearFunction::~LinearFunction() {
	// TODO Auto-generated destructor stub
}

double LinearFunction::getFx(double x){
    return m*x+b;
}
