#ifndef COLLISIONPATH_H_
#define COLLISIONPATH_H_
#include "SubPathPoint.h"

class CollisionPath {
public:
	CollisionPath();
    CollisionPath(SubPathPoint startPoint, SubPathPoint endPoint);
	virtual ~CollisionPath();

    SubPathPoint startPoint;
    SubPathPoint endPoint;

};

#endif /* COLLISIONPATH_H_ */
