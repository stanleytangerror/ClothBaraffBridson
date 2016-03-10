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

	void exportShearConditionData(float* & dataBuffer, GLuint & dataSize);
	void exportBendConditionData(float* & dataBuffer, GLuint & dataSize);

	Eigen::VectorXf const exportVelocity()
	{
		return v_total;
	}

	size_t exportVertexSize()
	{
		return VERTEX_SIZE;
	}

private:
	ClothPiece * const clothPiece;

	typedef Eigen::Triplet<float> Tri_float;

	// used for filling sparse matrix 
	// according to Filling a sparse matrix section of Eigen
	std::vector<Tri_float> coeff_list; 
	// used for reserve data in sparse matrix
	Eigen::VectorXi reserve_sparsematrix;

	// --------------- variables of model ------------------
	//PolyArrayMesh::Property_map<Veridx, Vec3f> vph_planarcoord;
	// WARNING access to map should via map.at()
	std::map<Veridx, GLuint> vertices2indices;
	std::map<Faceidx, GLuint> faces2indices;
	std::map<Edgeidx, GLuint> edges2indices;
	GLuint VERTEX_SIZE, FACE_SIZE, EDGE_SIZE;
	std::vector<float> mass_list;
	Eigen::VectorXf positions;

	// ----------------- variables needed by Baraff ---------------- 
	// parameters for integration
	float time_step;
	// parameters for cloth density
	float density = 0.1f;
	// parameters for stretch forces
	float k_stretch = 5e3f, kd_stretch = 0.2f;
	float bu = 20.0f, bv = 20.0f;
	// parameters for shear forces
	float k_shear = 5e2f, kd_shear = 0.2f;
	// parameters for bend forces
	float k_bend = 1e-5f, kd_bend = 0.2f;

	// vectors
	Eigen::VectorXf f_total; 
	Eigen::VectorXf v_total; 
	Eigen::VectorXf Cu_stretch, Cv_stretch; // condition per face
	Eigen::VectorXf C_shear; // condition per face
	Eigen::VectorXf C_bend; // condition per inner edge

	// matrices
	Eigen::SparseMatrix<float> df_dx_internal_total; // symmetric 
	Eigen::SparseMatrix<float> df_dx_damp_total; // symmetric 
	Eigen::SparseMatrix<float> df_dv_damp_total; // symmetric 
	Eigen::SparseMatrix<float> constraints;
	Eigen::SparseMatrix<float> mass_constrainted;
	// will not change once initialed
	Eigen::SparseMatrix<float> mass_inverse;
	Eigen::SparseMatrix<float> mass;
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

	void addConstraint(Veridx vhandle, Eigen::Vector3f direction);
	void addExternForce(Veridx vhandle, Eigen::Vector3f ext_force);

	void getStretchAndShearForce(Faceidx fhandle,
		const SurfaceMesh3f::Property_map<Veridx, Point3f> & vprop_planarcoord,
		float k_stretch, float kd_stretch, float k_shear, float kd_shear, float bu, float bv);

	void getBendForce(Faceidx fhandle0, Faceidx fhandle1, Edgeidx ehandle, float k_bend, float kd_bend);

};

#endif
