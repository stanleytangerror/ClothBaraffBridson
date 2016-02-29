#include "ConjugateGradientSolver.h"

#include <Eigen\IterativeLinearSolvers>

#include <iostream>

void testPCG()
{
	//Eigen::Vector3f du1(0.05f, 0.0f, 0.0f);
	//Eigen::Vector3f du2(0.05f, -0.05f, 0.0f);
	//GLfloat area = du1.cross(du2).norm();
	//std::cout << du1.cross(du2) << std::endl;
 //	std::cout << area << std::endl;
	//
	//Eigen::VectorXf v(20);
	//v.setZero();
	//std::cout << "v nonzero " << std::endl << v.nonZeros() << std::endl;
	//std::cout << "v block 0, 0 " << std::endl << v.block<3, 1>(0, 0) << std::endl;

	//GLfloat a = 1e10f;
	//for (size_t _i = 0; _i < 100; ++_i)
	//{
	//	a *= a;
	//	std::cout << "a " << std::endl << a << std::endl;
	//}


	Eigen::MatrixXf A(5, 5);
	A << 2.0f, 1.0f, 1.0f, 3.0f, 2.0f,
		1.0f, 2.0f, 2.0f, 1.0f, 1.0f,
		1.0f, 2.0f, 9.0f, 1.0f, 5.0f,
		3.0f, 1.0f, 1.0f, 7.0f, 1.0f,
		2.0f, 1.0f, 5.0f, 1.0f, 8.0f;
	Eigen::SparseMatrix<GLfloat> sA = A.sparseView();
	Eigen::VectorXf b(5);
	b << 11.0f, 3.0f, -5.0f, 5.0f, 27.0f;
	Eigen::VectorXf x(5);
	x << 3.0f, 2.0f, -4.0f, -1.0f, 5.0f;
	std::cout << "A * x" << std::endl << A * x << std::endl
		<< "b" << std::endl << b << std::endl
		<< "A" << std::endl << A << std::endl
		<< "x" << std::endl << x << std::endl;
	Eigen::SparseMatrix<GLfloat> S(5, 5);
	for (size_t _i = 0; _i < 5; ++_i)
		S.coeffRef(_i, _i) = 1.0f;

	//Eigen::ConjugateGradient<Eigen::SparseMatrix<GLfloat> > cg;
	//cg.compute(sA);
	//auto solution3 = cg.solve(b);
	//std::cout << solution3 << std::endl
	//	<< A * solution3 - b << std::endl;
	//std::cout << "#iterations:     " << cg.iterations() << std::endl;
	//std::cout << "estimated error: " << cg.error() << std::endl;

	ModifiedPCGSolver solver(sA, b, S);
	auto solution2 = solver.solve(0.01f, Eigen::VectorXf(b.size()));
	std::cout << solution2 << std::endl;
}
