#ifndef LINEARFUNCTION_H_
#define LINEARFUNCTION_H_

class LinearFunction {
public:
	LinearFunction(double m, double b);
	virtual ~LinearFunction();
    double m;
    double b;
    double getFx(double x);
};

#endif /* LINEARFUNCTION_H_ */
