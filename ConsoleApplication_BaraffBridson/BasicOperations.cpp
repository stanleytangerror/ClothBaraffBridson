#include "BasicOperations.h"

#include <iostream>

void convert_diag2sparse_mnf(Eigen::SparseMatrix<float> & dest, const Eigen::Diagonal<const Eigen::SparseMatrix<float>> & src)
{
	typedef Eigen::Triplet<float> Tri_float;
	static size_t size = src.size();
	//dest = Eigen::SparseMatrix<float>(size, size);
	std::vector<Tri_float> triples;
	triples.reserve(size);
	for (size_t _i = 0; _i < size; ++_i)
	{
		triples.push_back(Tri_float(_i, _i, src.coeff(_i)));
	}
	dest.setFromTriplets(triples.begin(), triples.end());
}

void get_diag_mnf(Eigen::SparseMatrix<float> & dest, size_t size)
{
	typedef Eigen::Triplet<float> Tri_float;
	std::vector<Tri_float> triples;
	triples.reserve(size);
	for (size_t _i = 0; _i < size; ++_i)
	{
		triples.push_back(Tri_float(_i, _i, 1.0f));
	}
	//dest = Eigen::SparseMatrix<float>(size, size);
	dest.setFromTriplets(triples.begin(), triples.end());
}

//#define DEBUG_OPERATIONS
void addBlock33(Eigen::SparseMatrix<float> & augend, GLuint block_i, GLuint block_j, const Eigen::Matrix3f & addend)
{
#ifdef DEBUG_OPERATIONS
	std::cout << "augend size " << augend.size() << std::endl;
	std::cout << "block " << block_i << ", " << block_j << std::endl;
	std::cout << "addend " << std::endl << addend << std::endl;
#endif

	for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
	{
		augend.coeffRef(block_i * 3 + _i, block_j * 3 + _j) += addend.coeff(_i, _j);
	}
}

void setBlock33(Eigen::SparseMatrix<float> & augend, GLuint block_i, GLuint block_j, const Eigen::Matrix3f & addend)
{
#ifdef DEBUG_OPERATIONS
	std::cout << "augend size " << augend.size() << std::endl;
	std::cout << "block " << block_i << ", " << block_j << std::endl;
	std::cout << "addend " << std::endl << addend << std::endl;
#endif

	for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
	{
		augend.coeffRef(block_i * 3 + _i, block_j * 3 + _j) = addend.coeff(_i, _j);
	}
}

//Eigen::Vector3f get_vector(Eigen::Tensor<float, 3>& tensor, GLuint block_i, GLuint block_j)
//{
//
//	return Eigen::Vector3f();
//}


GLboolean checkIdentical(const Eigen::Matrix3f mat1, const Eigen::Matrix3f mat2, float tolerance)
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

GLboolean checkSymmetrical(const Eigen::Matrix3f mat, float tolerance)
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

GLboolean checkSymmetrical(const Eigen::SparseMatrix<float> mat, float tolerance)
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

Eigen::Matrix3f get_S_m3f(Eigen::Vector3f & v)
{
	Eigen::Matrix3f S;
	S << 0.0f, -v[2], v[1],
		v[2], 0.0f, -v[0],
		-v[1], v[0], 0.0f;
	return S;
}

#ifdef OPENMESH_BASED
// xyz -> zxy
void shiftVertices(PolyArrayMesh::VertexHandle & vhd0, PolyArrayMesh::VertexHandle & vhd1, PolyArrayMesh::VertexHandle & vhd2)
{
	// x y z
	std::swap(vhd0, vhd1);
	// y x z
	std::swap(vhd0, vhd2);
	// z x y
}
#endif

#ifdef CGAL_BASED
// xyz -> zxy
void shiftVertices(Veridx & vhd0, Veridx & vhd1, Veridx & vhd2)
{
	// x y z
	std::swap(vhd0, vhd1);
	// y x z
	std::swap(vhd0, vhd2);
	// z x y
}
#endif

Eigen::Vector3f get_vector3f_block(Eigen::Tensor<float, 3> & tensor, unsigned int block_i, unsigned int block_j)
{
	Eigen::Vector3f vec;
	for (size_t _i = 0; _i < 3; ++_i)
	{
		vec[_i] = tensor.coeff(Eigen::array<Eigen::DenseIndex, 3U>(block_i, block_j, _i));
	}
	return vec;
}

