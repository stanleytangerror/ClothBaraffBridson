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

	void exportConditionData(GLfloat* & dataBuffer, GLuint & dataSize);
};

#endif
