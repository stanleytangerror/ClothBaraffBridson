#ifndef CONJUGATE_GRADIENT_SOLVER
#define CONJUGATE_GRADIENT_SOLVER


#include "BasicOperations.h"

#include <GL\glew.h>
#include <Eigen\Core>
#include <Eigen\Sparse>
#include <Eigen\IterativeLinearSolvers>

#include <iostream>

class ModifiedPCGSolver
{
public:
	ModifiedPCGSolver(Eigen::SparseMatrix<GLfloat> A, Eigen::VectorXf b, Eigen::SparseMatrix<GLfloat> S) :
		//A(A), b(b), S(S)
		A(A.transpose() * A), b(A.transpose() * b), S(S)
	{
		dim = this->b.innerSize();
		//std::cout << "dim " << dim << std::endl;
		this->P = Eigen::SparseMatrix<float>(dim, dim);
		convert_diag2sparse_mnf(this->P, this->A.diagonal());
		for (size_t _i = 0; _i < this->dim; ++_i)
		{
			if (this->P.coeff(_i, _i) == 0)
				this->P.coeffRef(_i, _i) = 1.0f;
		}
		P_inverse = this->P.cwiseInverse();
		initial();
	}

	const Eigen::VectorXf & solve(const GLfloat epsilon);

private:
	// dimension of solution vector
	size_t dim;
	// Ax = b in conjugate gradient method
	const Eigen::SparseMatrix<GLfloat> A;
	// Ax = b in conjugate gradient method
	const Eigen::VectorXf b;
	// preconditioning approximation of A in conjugate gradient method
	// P^{-1} approximate A
	Eigen::SparseMatrix<GLfloat> P;
	Eigen::SparseMatrix<GLfloat> P_inverse; 
	// Ax = b in conjugate gradient method
	Eigen::VectorXf x;

	// change in velocity in constraint directions
	Eigen::VectorXf z;
	// constraints of particles 
	Eigen::SparseMatrix<GLfloat> S;

	void initial();

	//GLboolean checkSymmetrical(GLfloat tolerance = 1e-20f);
	//GLboolean checkPositiveDefinite();

	inline Eigen::VectorXf filter(Eigen::VectorXf vec);

	inline GLfloat toScalar(const Eigen::SparseMatrix<GLfloat> product)
	{
		return (product.size() != (1, 1)) ? 0.0f : (Eigen::SparseMatrix<GLfloat>(product)).coeffRef(0, 0);
	}

	void runCG(const GLfloat epsilon);
	// baraff & witkin 98
	void runMPCG(const GLfloat epsilon);
	// Ascher & Boxerman 03
	void runProjectedCG(const GLfloat epsilon);
};


#endif