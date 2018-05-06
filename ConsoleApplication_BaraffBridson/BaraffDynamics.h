#ifndef BARAFF_DYNAMICS_H
#define BARAFF_DYNAMICS_H

#include "SurfaceMeshObject.h"
#include "BaraffPhysics.h"
#include "BaraffMPCGSolver.h"
#include "ContactHandler.h"

class BaraffDynamics
{
public:
	BaraffDynamics(SurfaceMeshObject * model, SurfaceMeshObject * collider) 
		: model(model)
		, physics(new BaraffPhysics(model))
		, mContactHandler(new ContactHandler(model, collider))
	{
		initial();
	}

	void stepforward(float time_step);

	void writeBack();

	SurfaceMeshObject* getClothPiece()
	{
		return model;
	}

	void exportShearConditionData(GLfloat* & dataBuffer, GLuint & dataSize);
	void exportBendConditionData(GLfloat* & dataBuffer, GLuint & dataSize);

	void RecomputeNormals();

private:
	SurfaceMeshObject* model;
	BaraffPhysics* physics;
	Eigen::VectorXf last_root;
	//BaraffMPCGSolver solver;

	ContactHandler * mContactHandler;

	void initial();

};

#endif
