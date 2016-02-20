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

	Eigen::SparseMatrix<float> exportA();
	Eigen::SparseMatrix<float> exportS();
	Eigen::VectorXf exportb();

	void compute(float time_step);
	void update(const Eigen::VectorXf & v_delta);
	void writeMesh();

	void exportConditionData(float* & dataBuffer, GLuint & dataSize);

private:
	ClothPiece* clothPiece;

	typedef Eigen::Triplet<float> Tri_float;

	std::vector<Tri_float> coeff_list; // used for filling sparse matrix 
										 // according to Filling a sparse matrix section of Eigen

	// --------------- variables of model ------------------
	OpenMesh::VPropHandleT<OpenMesh::Vec3f> vph_planarcoord;
	std::map<PolyArrayMesh::VertexHandle, GLuint> vertices2indices;
	std::map<PolyArrayMesh::FaceHandle, GLuint> faces2indices;
	GLuint VERTEX_SIZE, FACE_SIZE;
	std::vector<float> mass;
	Eigen::VectorXf positions;

	// ----------------- variables needed by Baraff ---------------- 
	// parameters for integration
	float time_step;
	// parameters for stretch forces
	float k_stretch = 5e3f, kd_stretch = 0.2f;
	float bu = 1.0f, bv = 1.0f;
	// parameters for shear forces
	float k_shear = 5e2f, kd_shear = 0.2f;
	// parameters for bend forces
	float k_bend = 1e1f, kd_bend = 0.2f;

	// vectors
	Eigen::VectorXf f_total; 
	Eigen::VectorXf v_total; 
	Eigen::VectorXf C_shear; // condition per face

	// matrices
	Eigen::SparseMatrix<float> df_dx_total; // symmetric 
	Eigen::SparseMatrix<float> df_dv_total; // symmetric 
	Eigen::SparseMatrix<float> constraints;
	Eigen::SparseMatrix<float> mass_constrainted;
	// will not change once initialed
	Eigen::SparseMatrix<float> mass_inverse;
	Eigen::SparseMatrix<float> identity;
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
		float k_stretch, float kd_stretch, float k_shear, float kd_shear, float bu, float bv);

	void getBendForce(PolyArrayMesh::FaceHandle fhandle0, PolyArrayMesh::FaceHandle fhandle1,
		PolyArrayMesh::EdgeHandle ehandle, float k_bend, float kd_bend);


};

#endif
