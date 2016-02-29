#ifndef SIMULATE_H
#define SIMULATE_H

#include "ClothPiece.h"
#include "BaraffRequire.h"
#include "ConjugateGradientSolver.h"


class Simulate
{
private:
	ClothPiece* model;
	BaraffRequire* variables;
	Eigen::VectorXf last_root;
	//ModifiedPCGSolver solver;

	void initial();
public:
	Simulate(ClothPiece * model) : model(model), variables(new BaraffRequire(model))
	{
		initial();
	}

	void simulate();

	void writeBack();

	ClothPiece* getClothPiece()
	{
		return model;
	}

	void exportShearConditionData(GLfloat* & dataBuffer, GLuint & dataSize);
	void exportBendConditionData(GLfloat* & dataBuffer, GLuint & dataSize);
};

#endif
