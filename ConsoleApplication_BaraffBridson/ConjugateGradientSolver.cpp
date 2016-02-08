#include "ConjugateGradientSolver.h"

#include "BasicOperations.h"

#include <iostream>


const Eigen::VectorXf & ModifiedPCGSolver::solve(const GLfloat epsilon)
{
	//std::cout << "A symmetrical " << (checkSymmetrical(A) ? true : false) << std::endl;
	//std::cout << "x before " << std::endl << x << std::endl;
	runMPCG(epsilon);
	//std::cout << "x after " << std::endl << x << std::endl;
	return x;
}

void ModifiedPCGSolver::initial()
{
	x = Eigen::VectorXf(dim);
	x.setZero();
	// DEPRECATED
	z = Eigen::VectorXf(dim);
	z.setZero();
	// DEPRECATED
	//P_inverse = Eigen::SparseMatrix<GLfloat>(dim, dim);
	//for (size_t _i = 0; _i < dim; ++_i)
	//	P_inverse.coeffRef(_i, _i) = A.coeff(_i, _i);
	//P = P_inverse.cwiseInverse();
	//P = P_inverse.cwiseInverse();
	//P_inverse = A;
	//P = 

	//std::cout <<"dim = " << dim << std::endl
	//	<< "A" << std::endl << A << std::endl
	//	<< "x" << std::endl << x << std::endl
	//	<< "b" << std::endl << b << std::endl
	//	<< "P" << std::endl << P << std::endl
	//	<< "P_inverse" << std::endl << P_inverse << std::endl
	//	<< "z" << std::endl << z << std::endl
	//	<< "S" << std::endl << S << std::endl
	//	<< std::endl;

}

inline Eigen::VectorXf ModifiedPCGSolver::filter(Eigen::VectorXf vec)
{
	return S * vec;
}

#define SHOW_CONVERGENCE

void ModifiedPCGSolver::runCG(const GLfloat epsilon)
{
#ifdef SHOW_CONVERGENCE
	std::cout << " ========== CG ============== " << std::endl;
#endif
	Eigen::VectorXf residual(dim), direction(dim), span_direction(dim), q(dim);
	GLfloat alpha, delta_0, delta_old, delta_new;
	// initial solution
	x = z;
	residual = b - A * x;
	direction = residual;
	delta_0 = residual.transpose() * residual;
	delta_new = delta_0;
#ifdef SHOW_CONVERGENCE
	std::cout << "initial value " << delta_new << std::endl;
	std::cout << "target value " << delta_0 << std::endl;
#endif
	size_t restart_round = 0;
	while (delta_new > epsilon * epsilon * delta_0)
	{
		size_t round = 0;
		//std::cout << "restart" << std::endl;
		while (round < dim && delta_new > epsilon * epsilon * delta_0)
		{
			q = A * direction;
			alpha = delta_new / (direction.transpose() * q);
			x = x + alpha * direction;
			residual -= alpha * q;
			//span_direction = P_inverse * residual;
			delta_old = delta_new;
			delta_new = residual.transpose() * residual;
			direction = residual + (delta_new / delta_old) * direction;
			round += 1;
#ifdef SHOW_CONVERGENCE
			std::cout << "round " << round << std::endl
				<< "delta_new " << delta_new << std::endl
				<< "epsilon^2 * delta_0 " << epsilon * epsilon * delta_0 << std::endl
				//<< "x " << std::endl << x << std::endl
				//<< "residual " << std::endl << residual << std::endl
				;
#endif
		}
		restart_round += 1;
#ifdef SHOW_CONVERGENCE
		std::cout << " ---- restart_round " << restart_round << " ---- " << std::endl
			<< "delta_new " << delta_new << ", "
			<< "epsilon^2 * delta_0 " << epsilon * epsilon * delta_0 << std::endl;
#endif
	}
}

void ModifiedPCGSolver::runMPCG(const GLfloat epsilon)
{
#ifdef SHOW_CONVERGENCE
	std::cout << " ========== MPCG ============== " << std::endl;
#endif
	Eigen::VectorXf residual(dim), direction(dim), span_direction(dim), q(dim);
	GLfloat alpha, delta_0, delta_old, delta_new, end_point = 1e-2f;
	// !!!
	x = z;
	//Eigen::VectorXf b_filter = filter(b);
	//delta_0 = b_filter.transpose() * P * b_filter;
	//std::cout << "size " << std::endl
	//	<< "A " << A.size() << std::endl
	//	<< "b " << b.size() << std::endl
	//	<< "x " << x.size() << std::endl;
	//std::cout << "x " << std::endl << x << std::endl;
	//std::cout << "A " << std::endl << A << std::endl;
	//std::cout << "b " << std::endl << b << std::endl;
	residual = (b - A * x);
	//std::cout << "residual " << std::endl << residual << std::endl;
	//residual = filter(b - A * x);
	direction = (P_inverse * residual);
	//direction = filter(P_inverse * residual);
	delta_new = residual.transpose() * direction;
	delta_0 = delta_new;
	// !!!
	end_point = epsilon * epsilon * delta_0;
#ifdef SHOW_CONVERGENCE
	std::cout << "initial value " << delta_new << std::endl;
	std::cout << "target value " << end_point << std::endl;
#endif
	//std::cout << residual << std::endl;
	//std::cout << direction << std::endl;
	size_t restart_round = 0;
	// !!!
	while (delta_new > end_point)
	{
		size_t round = 0;
		residual = (b - A * x);
		direction = (P_inverse * residual);
		//std::cout << "restart" << std::endl;
		while (round < dim && delta_new > end_point) {
			//std::cout << "delta_new " << delta_new << std::endl;
			//std::cout << "delta_0 " << delta_0 << std::endl;
			q = (A * direction);
			//q = filter(A * direction);
			alpha = delta_new / (direction.transpose() * q);
			//std::cout << "alpha " << alpha << std::endl;
			x = x + alpha * direction;
			// !!!
			residual = b - A * x;
			//residual = residual - alpha * q;
			//std::cout << "residual " << std::endl << residual << std::endl;
			span_direction = P_inverse * residual;
			delta_old = delta_new;
			delta_new = residual.transpose() * span_direction;
			direction = (span_direction + delta_new / delta_old * direction);
			//direction = filter(span_direction + delta_new / delta_old * direction);
			round += 1;
#ifdef SHOW_CONVERGENCE
			std::cout << "round " << round << std::endl
				<< "delta_new " << delta_new << ", "
				<< "epsilon^2 * delta_0 " << epsilon * epsilon * delta_0 << std::endl
				//<< "x " << std::endl << x << std::endl
				//<< "residual " << std::endl << residual << std::endl
				;
#endif
		}
		restart_round += 1;
#ifdef SHOW_CONVERGENCE
		std::cout << " ---- restart_round " << restart_round << " ---- " << std::endl
			<< "delta_new " << delta_new << ", "
			<< "epsilon^2 * delta_0 " << epsilon * epsilon * delta_0 << std::endl;
#endif
	}
	//std::cout << "x " << std::endl << x << std::endl;
}

void ModifiedPCGSolver::runProjectedCG(const GLfloat epsilon)
{
#ifdef SHOW_CONVERGENCE
	std::cout << " ========== Projected CG ============== " << std::endl;
#endif
	Eigen::VectorXf b_hat(dim), residual(dim), p(dim), s(dim), h(dim);
	Eigen::SparseMatrix<GLfloat> iden, iden_minus_S;
	GLfloat b_delta, delta, delta_hat, alpha;
	get_diag_mnf(iden, dim);
	iden_minus_S = iden - S;
	x = z;
	//x = iden_minus_S * z;
	b_hat = S * (b);
	//b_hat = S * (b - A * iden_minus_S * z);
	b_delta = b_hat.transpose() * P_inverse * b_hat;
	residual = S * (b - A * x);
	p = S * P_inverse * residual;
	delta = residual.transpose() * p;
	size_t restart_round = 0;
	while (delta > epsilon * epsilon * b_delta)
	{
		size_t round = 0;
		//std::cout << "restart" << std::endl;
		while (round < dim && delta > epsilon * epsilon * b_delta) {
			//if (round == 87)
			//	std::cout << "round 87" << std::endl;
			s = S * A * p;
			alpha = delta / (p.transpose() * s);
			x = x + alpha * p;
			residual = residual - alpha * s;
			h = P_inverse * residual;
			delta_hat = delta;
			delta = residual.transpose() * h;
			p = S * (h + (delta / delta_hat) * p);
			round += 1;
#ifdef SHOW_CONVERGENCE
			std::cout << "round " << round << std::endl
				<< "delta " << delta << ", "
				<< "delta_hat " << delta_hat << ", "
				<< "epsilon^2 * b_delta " << epsilon * epsilon * b_delta << std::endl
				//<< "x " << std::endl << x << std::endl
				//<< "residual " << std::endl << residual.norm() << std::endl
				;
#endif
		}
		restart_round += 1;
#ifdef SHOW_CONVERGENCE
		std::cout << "round " << round << std::endl
			<< "delta " << delta << ", "
			<< "epsilon^2 * b_delta " << epsilon * epsilon * b_delta << std::endl;
#endif
	}
}
