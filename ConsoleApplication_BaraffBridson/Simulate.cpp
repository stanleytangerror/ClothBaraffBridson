#include "Simulate.h"

#include "BasicOperations.h"

#include <Eigen/IterativeLinearSolvers>
#include <iostream>

void Simulate::initial()
{
	//variables = BaraffRequire(model);
}

void Simulate::simulate()
{
	variables->compute(0.2f);
	Eigen::SparseMatrix<GLfloat> A = variables->exportA();
	Eigen::VectorXf b = variables->exportb();
	Eigen::SparseMatrix<GLfloat> S = variables->exportS();
	//std::cout << "size " << std::endl
	//	<< "A " << A.size() << std::endl
	//	<< "b " << b.size() << std::endl
	//	<< "S " << S.size() << std::endl
	//<< "A " << A.nonZeros() << std::endl
	//<< "b " << b.nonZeros() << std::endl
	//<< "S " << S.nonZeros() << std::endl;
	Eigen::VectorXf v_delta;

//#define PCG_EIGEN
#define MPCG_BARAFF

#ifdef PCG_EIGEN
	Eigen::SparseMatrix<GLfloat> temp1 = A.transpose() * A;
	std::cout << "ATA sym " << (checkSymmetrical(temp1) ? true : false) << std::endl;
	Eigen::ConjugateGradient<Eigen::SparseMatrix<GLfloat> > solver1;
	solver1.compute(temp1);
	Eigen::VectorXf temp2 = A.transpose() * b;
	Eigen::VectorXf v_delta1 = solver1.solve(temp2);
#endif

#ifdef MPCG_BARAFF
	ModifiedPCGSolver solver2 = ModifiedPCGSolver(A, b, S);
	Eigen::VectorXf v_delta2 = solver2.solve(0.05f);
#endif

#ifdef MPCG_BARAFF 
#ifdef PCG_EIGEN
	Eigen::VectorXf bias = v_delta1 - v_delta2;
	std::cout << "bias " << std::endl << bias << std::endl;
	std::cout << "bias norm " << bias.norm() << std::endl;
	std::cout << "bias range " << bias.minCoeff() << ", " << bias.maxCoeff() << std::endl;
#endif 
#endif

#ifdef DEBUG_SOLVE_EQUATION
	std::cout << "solution " << std::endl << v_delta2 << std::endl;
#endif
	variables->update(v_delta2);
}

void Simulate::writeBack()
{
	variables->writeMesh();
}

void Simulate::exportShearConditionData(GLfloat *& dataBuffer, GLuint & dataSize)
{
	variables->exportShearConditionData(dataBuffer, dataSize);
}

void Simulate::exportBendConditionData(GLfloat *& dataBuffer, GLuint & dataSize)
{
	// TODO
	variables->exportBendConditionData(dataBuffer, dataSize);
}

