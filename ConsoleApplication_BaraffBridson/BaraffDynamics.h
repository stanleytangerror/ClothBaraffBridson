#ifndef SIMULATE_H
#define SIMULATE_H

#include "ClothPiece.h"
#include "BaraffPhysics.h"
#include "BaraffMPCGSolver.h"


class BaraffDynamics
{

public:
	BaraffDynamics(ClothPiece * model) : 
		model(model), physics(new BaraffPhysics(model))
	{
		initial();
	}

	void stepforward(float time_step);

	void writeBack();

	ClothPiece* getClothPiece()
	{
		return model;
	}

	void exportShearConditionData(GLfloat* & dataBuffer, GLuint & dataSize);
	void exportBendConditionData(GLfloat* & dataBuffer, GLuint & dataSize);

private:
	ClothPiece* model;
	BaraffPhysics* physics;
	Eigen::VectorXf last_root;
	//BaraffMPCGSolver solver;

	void initial();

};

#endif
