#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "BasicTypes.h"
#include <Eigen/Dense>

class Constraint
{
public:
	//unsigned int m_numberOfBodies;
	/** indices of the linked bodies */
	//unsigned int *m_bodies;

	Constraint(/*const unsigned int numberOfBodies*/)
	{
		//m_numberOfBodies = numberOfBodies;
		//m_bodies = new unsigned int[numberOfBodies];
	}

	virtual ~Constraint() { /*delete[] m_bodies;*/ };
	virtual int &getTypeId() const = 0;

	virtual bool updateConstraint() = 0;
	virtual bool solvePositionConstraint() = 0;
	virtual bool solveVelocityConstraint() = 0;
};

//class PointTriangleCollisionConstraint : public Constraint
//{
//	PointTriangleCollisionConstraint(Eigen::Vector3f & point, Triangle3f & triangle, float thickness)
//		: point(point), triangle(triangle), thickness(thickness) 
//	{}
//private:
//	Point3f & point;
//	Triangle3f & triangle;
//	float thickness;
//
//};

#endif
