#include "BaraffDynamics.h"

#include "BasicOperations.h"

#include <Eigen/IterativeLinearSolvers>
#include <iostream>

void BaraffDynamics::initial()
{
	//physics = BaraffPhysics(model);
	last_root = Eigen::VectorXf::Zero(3 * physics->exportVertexSize());
}

void BaraffDynamics::stepforward(float time_step)
{
	physics->compute(time_step);
	Eigen::SparseMatrix<GLfloat> A = physics->exportA();
	Eigen::VectorXf b = physics->exportb();
	Eigen::SparseMatrix<GLfloat> S = physics->exportS();
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
	BaraffMPCGSolver solver2 = BaraffMPCGSolver(A, b, S);
	//Eigen::VectorXf v_delta2 = solver2.solve(0.05f, last_root);
	Eigen::VectorXf v_delta2 = solver2.solve(0.05f, Eigen::VectorXf::Zero(physics->exportVertexSize() * 3));
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
	last_root = v_delta2;
	physics->update(v_delta2);
}

void BaraffDynamics::writeBack()
{
	physics->writeMesh();
}

void BaraffDynamics::exportShearConditionData(GLfloat *& dataBuffer, GLuint & dataSize)
{
	physics->exportShearConditionData(dataBuffer, dataSize);
}

void BaraffDynamics::exportBendConditionData(GLfloat *& dataBuffer, GLuint & dataSize)
{
	// TODO
	physics->exportBendConditionData(dataBuffer, dataSize);
}

