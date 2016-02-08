#ifndef BASIC_OPERATIONS_H
#define BASIC_OPERATIONS_H

#include <GL/glew.h>

#include <Eigen\Core>
#include <Eigen\Sparse>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>


inline void copy_v3f(Eigen::Vector3f & dest, const OpenMesh::Vec3f & src)
{
	dest(0) = src[0];
	dest(1) = src[1];
	dest(2) = src[2];
}

inline void copy_v3f(OpenMesh::Vec3f & dest, const Eigen::Vector3f & src)
{
	dest[0] = src(0);
	dest[1] = src(1);
	dest[2] = src(2);
}

void convert_diag2sparse_mnf(Eigen::SparseMatrix<GLfloat> & dest, const Eigen::Diagonal<const Eigen::SparseMatrix<GLfloat>> const & src);

void addBlock33(Eigen::SparseMatrix<GLfloat> & augend, GLuint block_i, GLuint block_j, const Eigen::Matrix3f & addend);

void get_diag_mnf(Eigen::SparseMatrix<GLfloat> & dest, size_t size);

inline GLfloat max(GLfloat a, GLfloat b)
{
	return (a > b) ? a : b;
}

inline GLfloat min(GLfloat a, GLfloat b)
{
	return (a < b) ? a : b;
}

GLboolean checkIdentical(const Eigen::Matrix3f mat1, const Eigen::Matrix3f mat2, GLfloat tolerance = 1e-20f);

GLboolean checkSymmetrical(const Eigen::Matrix3f mat, const GLfloat tolerance = 1e-20f);

GLboolean checkSymmetrical(const Eigen::SparseMatrix<GLfloat> mat, GLfloat tolerance = 1e-20f);


#endif

