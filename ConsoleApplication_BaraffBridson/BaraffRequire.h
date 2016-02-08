#ifndef BARAFF_REQUIRE
#define BARAFF_REQUIRE

#include "ClothPiece.h"

#include <Eigen\Core>
#include <Eigen\Dense>
#include <Eigen\Sparse>

class BaraffRequire
{
public:
	BaraffRequire(ClothPiece* clothPiece):
		clothPiece(clothPiece) 
	{
		initial();
	}

	Eigen::SparseMatrix<GLfloat> exportA();
	Eigen::SparseMatrix<GLfloat> exportS();
	Eigen::VectorXf exportb();

	void compute(GLfloat time_step);
	void update(const Eigen::VectorXf & v_delta);
	void writeMesh();

	void exportConditionData(GLfloat* & dataBuffer, GLuint & dataSize);

private:
	ClothPiece* clothPiece;

	typedef Eigen::Triplet<GLfloat> Tri_GLfloat;

	std::vector<Tri_GLfloat> coeff_list; // used for filling sparse matrix 
										 // according to Filling a sparse matrix section of Eigen

	// --------------- variables of model ------------------
	OpenMesh::VPropHandleT<OpenMesh::Vec3f> vph_planarcoord;
	std::map<PolyArrayMesh::VertexHandle, GLuint> vertices2indices;
	std::map<PolyArrayMesh::FaceHandle, GLuint> faces2indices;
	GLuint VERTEX_SIZE, FACE_SIZE;
	std::vector<GLfloat> mass;
	Eigen::VectorXf positions;

	// ----------------- variables needed by Baraff ---------------- 
	// parameters for integration
	GLfloat time_step;
	// parameters for stretch forces
	GLfloat k_stretch = 5e3f, kd_stretch = 0.2f;
	GLfloat bu = 20.0f, bv = 20.0f;
	// parameters for shear forces
	GLfloat k_shear = 5e2f, kd_shear = 0.2f;
	// parameters for bend forces
	// ...

	// vectors
	Eigen::VectorXf f_total; 
	Eigen::VectorXf v_total; 
	Eigen::VectorXf C_shear; // condition per face

	// matrices
	Eigen::SparseMatrix<GLfloat> df_dx_total; // symmetric 
	Eigen::SparseMatrix<GLfloat> df_dv_total; // symmetric 
	Eigen::SparseMatrix<GLfloat> constraints;
	Eigen::SparseMatrix<GLfloat> mass_constrainted;
	// will not change once initialed
	Eigen::SparseMatrix<GLfloat> mass_inverse;
	Eigen::SparseMatrix<GLfloat> identity;
	//Eigen::DiagonalMatrix<int, Eigen::Dynamic> diagonal_matrix;


	// -------------- pipelines between model and variables ----------------- 
	void readPositions();
	void writePositions();

	// -------------- compute functions ----------------- 
	/* called once, used for allocating memory */
	void initial();

	// called before each iteration
	void reset(GLboolean first);

	void addConstraint(PolyArrayMesh::VertexHandle vhandle, Eigen::Vector3f direction);
	void addExternForce(PolyArrayMesh::VertexHandle vhandle, Eigen::Vector3f ext_force);

	void getStretchAndShearForce(PolyArrayMesh::FaceHandle fhandle,
		const OpenMesh::VPropHandleT<OpenMesh::Vec3f> & vprop_planarcoord,
		GLfloat k_stretch, GLfloat kd_stretch, GLfloat k_shear, GLfloat kd_shear, GLfloat bu, GLfloat bv);

	void getBendForce(PolyArrayMesh::FaceHandle fhandle0, PolyArrayMesh::FaceHandle fhandle1,
		PolyArrayMesh::EdgeHandle ehandle, const OpenMesh::VPropHandleT<OpenMesh::Vec3f> & vprop_planarcoord,
		GLfloat k_bend, GLfloat kd_bend);


};

#endif
