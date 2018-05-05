#include "BaraffDynamics.h"

#include "BasicOperations.h"
#include "Profile.h"
#include <Eigen/IterativeLinearSolvers>
#include <iostream>

void BaraffDynamics::initial()
{
	//physics = BaraffPhysics(model);
	last_root = Eigen::VectorXf::Zero(3 * model->getVertexSize());
}

void BaraffDynamics::stepforward(float time_step)
{
	Temp::ProfileNewFrame();

	{
		PROFILE_SCOPE;
		physics->compute(time_step);
	}

	Eigen::SparseMatrix<GLfloat> A;
	Eigen::VectorXf b;
	Eigen::SparseMatrix<GLfloat> S;

	{
		PROFILE_SCOPE;

		A = physics->exportA();
		b = physics->exportb();
		S = physics->exportS();
	}
	
	Eigen::VectorXf v_delta;
	Eigen::VectorXf v_delta2;

	//#define PCG_EIGEN
#define MPCG_BARAFF

	{
		PROFILE_SCOPE;

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
		v_delta2 = solver2.solve(0.05f, Eigen::VectorXf::Zero(model->getVertexSize() * 3));
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
	}

	{
		PROFILE_SCOPE;
		last_root = v_delta2;
		physics->update(v_delta2);
	}
}

void BaraffDynamics::writeBack()
{
	model->setPositions(physics->exportPosition());
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

void BaraffDynamics::RecomputeNormals()
{
	model->RecomputeNormals();
}

