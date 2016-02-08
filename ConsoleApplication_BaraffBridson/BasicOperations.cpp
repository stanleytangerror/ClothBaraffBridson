#include "BasicOperations.h"

#include <iostream>

void convert_diag2sparse_mnf(Eigen::SparseMatrix<GLfloat> & dest, const Eigen::Diagonal<const Eigen::SparseMatrix<GLfloat>> const & src)
{
	typedef Eigen::Triplet<GLfloat> Tri_GLfloat;
	static size_t size = src.size();
	dest = Eigen::SparseMatrix<GLfloat>(size, size);
	std::vector<Tri_GLfloat> triples;
	triples.reserve(size);
	for (size_t _i = 0; _i < size; ++_i)
	{
		triples.push_back(Tri_GLfloat(_i, _i, src.coeff(_i)));
	}
	dest.setFromTriplets(triples.begin(), triples.end());
}

void get_diag_mnf(Eigen::SparseMatrix<GLfloat> & dest, size_t size)
{
	typedef Eigen::Triplet<GLfloat> Tri_GLfloat;
	std::vector<Tri_GLfloat> triples;
	triples.reserve(size);
	for (size_t _i = 0; _i < size; ++_i)
	{
		triples.push_back(Tri_GLfloat(_i, _i, 1.0f));
	}
	dest = Eigen::SparseMatrix<GLfloat>(size, size);
	dest.setFromTriplets(triples.begin(), triples.end());
}

void addBlock33(Eigen::SparseMatrix<GLfloat> & augend, GLuint block_i, GLuint block_j, const Eigen::Matrix3f & addend)
{
	for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
	{
		augend.coeffRef(block_i * 3 + _i, block_j * 3 + _j) += addend.coeff(_i, _j);
	}
}


GLboolean checkIdentical(const Eigen::Matrix3f mat1, const Eigen::Matrix3f mat2, GLfloat tolerance)
{
	int outer_size = mat1.outerSize();
	int inner_size = mat2.innerSize();
	if (outer_size != mat2.outerSize() || inner_size != mat2.innerSize())
		return false;
	GLboolean result = true;
	for (size_t _i = 0; _i < outer_size; ++_i) for (size_t _j = 0; _j < inner_size; ++_j)
		if (fabs(mat1.coeff(_i, _j) - mat2.coeff(_j, _i)) > tolerance)
		{
			std::cout << "diff [" << _i << "][" << _j << "] " << fabs(mat1.coeff(_i, _j) - mat2.coeff(_j, _i)) << std::endl;
			result = false;
		}
	return result;
}

GLboolean checkSymmetrical(const Eigen::Matrix3f mat, GLfloat tolerance)
{
	int outer_size = mat.outerSize();
	int inner_size = mat.innerSize();
	if (outer_size != inner_size)
		return false;
	GLboolean result = true;
	for (size_t _i = 0; _i < outer_size - 1; ++_i) for (size_t _j = _i + 1; _j < inner_size; ++_j)
		if (fabs(mat.coeff(_i, _j) - mat.coeff(_j, _i)) > tolerance)
		{
			std::cout << "diff [" << _i << "][" << _j << "] " << fabs(mat.coeff(_i, _j) - mat.coeff(_j, _i)) << std::endl;
			result = false;
		}
	return result;
}

GLboolean checkSymmetrical(const Eigen::SparseMatrix<GLfloat> mat, GLfloat tolerance)
{
	int outer_size = mat.outerSize();
	int inner_size = mat.innerSize();
	if (outer_size != inner_size)
		return false;
	GLboolean result = true;
	for (size_t _i = 0; _i < outer_size - 1; ++_i) for (size_t _j = _i + 1; _j < inner_size; ++_j)
		if (fabs(mat.coeff(_i, _j) - mat.coeff(_j, _i)) > tolerance)
		{
			std::cout << "diff [" << _i << "][" << _j << "] " << fabs(mat.coeff(_i, _j) - mat.coeff(_j, _i)) << std::endl;
			result = false;
		}
	return result;
}


