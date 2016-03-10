#include "BaraffRequire.h"
#include "BasicOperations.h"
#include "Types.h"
//#include "Tensor3.h"

#include <Eigen\Core>
#include <unsupported\Eigen\CXX11\Tensor>

#include <iostream>

#define STRETCH_FORCE
#define SHEAR_FORCE
#define BEND_FORCE

#define USE_DAMP
#define USE_STRETCH_DAMP
#define USE_SHEAR_DAMP
#define USE_BEND_DAMP

#define USE_GRAVITY
#define USE_CONSTRAINTS

//#define USE_NEW_A_b

#define DEBUG_ENERGY
//#define DEBUG_FORCE



typedef Eigen::Tensor<float, 3> Tensor3f;
typedef Eigen::array<Eigen::DenseIndex, 3U> index3;

Eigen::SparseMatrix<float> BaraffRequire::exportA()
{
	//Eigen::SparseMatrix<float> temp1 = mass_constrainted * df_dv_total;
	//std::cout << "mass_constrainted * df_dv_total diagonal" << std::endl << temp1.diagonal() << std::endl;
	//std::cout << "mass_constrainted * df_dv_total " << std::endl << temp1.size() << std::endl;
	//Eigen::SparseMatrix<float> temp2 = mass_constrainted * df_dx_total;
	//std::cout << "mass_constrainted * df_dx_total diagonal" << std::endl << temp2.diagonal() << std::endl;
	//std::cout << "mass_constrainted * df_dx_total " << std::endl << temp2.size() << std::endl;

	//std::cout << "mass_constrainted diagonal" << std::endl << mass_constrainted.diagonal() << std::endl;
	//std::cout << "mass_constrainted " << std::endl << mass_constrainted.size() << std::endl;
	//std::cout << "df_dx_total diagonal" << std::endl << df_dx_total.diagonal() << std::endl;
	//std::cout << "df_dx_total " << std::endl << df_dx_total.size() << std::endl;
	//std::cout << "df_dv_total diagonal" << std::endl << df_dv_total.diagonal() << std::endl;
	//std::cout << "df_dv_total " << std::endl << df_dv_total.size() << std::endl;

	//std::cout << "mass_constrainted " << (checkSymmetrical(mass_constrainted) ? true : false) << std::endl;
	//std::cout << "df_dx_total " << (checkSymmetrical(df_dx_total) ? true : false) << std::endl;
	//std::cout << "df_dv_total " << (checkSymmetrical(df_dv_total) ? true : false) << std::endl;

	//Eigen::SparseMatrix<float> val1 = (mass_constrainted * df_dv_total);
	//std::cout << "val1 " << (checkSymmetrical(val1) ? true : false) << std::endl;
	//Eigen::SparseMatrix<float> val2 = (mass_constrainted * df_dx_total);
	//std::cout << "val2 " << (checkSymmetrical(val2) ? true : false) << std::endl;
	//Eigen::SparseMatrix<float> val3 = val1 + time_step * val2;
	//std::cout << "val3 " << (checkSymmetrical(val3) ? true : false) << std::endl;
	//Eigen::SparseMatrix<float> val4 = (-time_step) * val3 + identity;
	
#ifdef USE_NEW_A_b
	Eigen::SparseMatrix<float> A = mass - time_step * df_dv_total - time_step * time_step * df_dx_total;
#else
	Eigen::SparseMatrix<float> A = identity - time_step * (mass_constrainted * df_dv_damp_total)
		- time_step * time_step * (mass_constrainted * (df_dx_internal_total + df_dx_damp_total));
#endif
	//std::cout << "A " << (checkSymmetrical(A, 1e-6) ? true : false) << std::endl;
	//std::cout << "A diagonal " << std::endl << A.diagonal() << std::endl;
	//std::cout << "A nonzero " << std::endl << A.nonZeros() << std::endl;
	return A;
}

Eigen::SparseMatrix<float> BaraffRequire::exportS()
{
	return constraints;
}

Eigen::VectorXf BaraffRequire::exportb()
{

	//f_total.pruned(1e-20f);
	//std::cout << "f_total " << std::endl << f_total << std::endl;
	//std::cout << "v_total " << std::endl << v_total << std::endl;
	//std::cout << "time_step " << std::endl << time_step << std::endl;
	//std::cout << "mass_constrainted " << std::endl << mass_constrainted << std::endl;
	//std::cout << "df_dx_total " << std::endl << df_dx_total << std::endl;

	//std::cout << "f_total " << std::endl << f_total.size() << std::endl;
	//std::cout << "v_total " << std::endl << v_total.size() << std::endl;
	//std::cout << "mass_constrainted " << std::endl << mass_constrainted.size() << std::endl;
	//std::cout << "df_dx_total " << std::endl << df_dx_total.size() << std::endl;
	//
	//auto temp1 = df_dx_total * v_total;
	//std::cout << "df_dx_total * v_total " << std::endl << temp1 << std::endl;
	//std::cout << "df_dx_total * v_total " << std::endl << temp1.size() << std::endl;
	//auto temp2 = temp1 * time_step;
	//std::cout << "df_dx_total * v_total * time_step " << std::endl << temp2 << std::endl;
	//std::cout << "df_dx_total * v_total * time_step " << std::endl << temp2.size() << std::endl;
	//auto temp3 = (f_total + temp2);
	//std::cout << "(f_total + df_dx_total * v_total * time_step) " << std::endl << temp3 << std::endl;
	//std::cout << "(f_total + df_dx_total * v_total * time_step) " << std::endl << temp3.size() << std::endl;

#ifdef USE_NEW_A_b
	Eigen::VectorXf b = time_step * (f_total + time_step * df_dx_total * v_total);
#else
	Eigen::VectorXf b = mass_constrainted * (f_total + (df_dx_internal_total + df_dx_damp_total) * v_total * time_step) * time_step;
#endif

	//std::cout << "b " << std::endl << b << std::endl;
	//std::cout << "b " << std::endl << b.size() << std::endl;

	return b;
}

void BaraffRequire::compute(float time_step)
{
	reset(false);

	this->time_step = time_step;

	SurfaceMesh3f* mesh = clothPiece->getMesh();

#ifdef USE_CONSTRAINTS
	// add constraints
	auto iter = mesh->vertices_begin();
	Veridx vh = *iter++;
	addConstraint(vh, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	addConstraint(vh, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	addConstraint(vh, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	vh = *iter++;
	addConstraint(vh, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	addConstraint(vh, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	addConstraint(vh, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	//for (size_t _i = 0; _i < 8; ++_i, ++iter)
	//{
	//	vh = *iter;
	//	addConstraint(vh, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	//	addConstraint(vh, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	//	addConstraint(vh, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	//}
	//for (size_t _i = 0; _i < 100; ++_i, ++iter)
	//{
	//	vh = *iter;
	//	addConstraint(vh, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	//	addConstraint(vh, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	//	addConstraint(vh, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	//}
	//for (size_t _i = 0; _i < 9; ++_i, ++iter);
	//vh = *iter;
	//addConstraint(vh, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	//addConstraint(vh, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	//addConstraint(vh, Eigen::Vector3f(1.0f, 0.0f, 0.0f));


#endif

	// add stretch and shear forces
#if defined(STRETCH_FORCE) || defined(SHEAR_FORCE)
	// forces, velocities and their derivatives
	//std::cout << "f_total without stretch " << std::endl << f_total << std::endl;
	//std::cout << "df_dx_total without stretch " << std::endl << df_dx_total << std::endl;
	//std::cout << "out v_total " << std::endl << v_total << std::endl;
	//std::cout << "out v_total nonzero " << std::endl << v_total.nonZeros() << std::endl;
	//std::cout << "out v_total[0] " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;
	size_t face_cnt = 0;
	SurfaceMesh3f::Property_map<Veridx, Point3f> vph_planarcoord = 
		mesh->property_map<Veridx, Point3f>(clothPiece->pname_vertexPlanarCoords).first;
	BOOST_FOREACH(Faceidx fhd, mesh->faces())
	{
		getStretchAndShearForce(fhd, vph_planarcoord, k_stretch, kd_stretch, k_shear, kd_shear, bu, bv);
		face_cnt += 1;
		//break;
	}

#ifdef DEBUG_ENERGY
	std::cout << "stretch energy " << Cu_stretch.norm() + Cv_stretch.norm() << std::endl;
	std::cout << "shear energy " << C_shear.norm() << std::endl;
#endif
	//std::cout << "C_shear " << std::endl << C_shear << std::endl;
	//std::cout << "C_shear nonzero " << C_shear.nonZeros() << std::endl;
	//std::cout << "C_shear norm " << C_shear.norm() << std::endl;
	//float magnitude = max(fabs(C_shear.maxCoeff()), fabs(C_shear.minCoeff()));
	//Eigen::VectorXf temp = C_shear / magnitude;
	//std::cout << "C_shear absmax " << magnitude << std::endl;
	//std::cout << "C_shear norm absmax " << max(fabs(temp.maxCoeff()), fabs(temp.minCoeff())) << std::endl;
	//std::cout << "f_total with stretch " << std::endl << f_total << std::endl;
	//std::cout << "f_total with stretch block " << std::endl << f_total.block<30, 1>(0, 0) << std::endl;
	//std::cout << "face_cnt " << std::endl << face_cnt << std::endl;
	//std::cout << "df_dx_total with stretch " << std::endl << df_dx_total << std::endl;
#endif

	// add bend force
#ifdef BEND_FORCE
	BOOST_FOREACH (Edgeidx ehd, mesh->edges())
	{
		Faceidx fhd0, fhd1;
		fhd0 = mesh->face(mesh->halfedge(ehd, 0));
		fhd1 = mesh->face(mesh->halfedge(ehd, 1));
		if (!fhd0.is_valid() || !fhd1.is_valid() || fhd0 == fhd1) continue;
		getBendForce(fhd0, fhd1, ehd, k_bend, kd_bend);
	}
#ifdef DEBUG_ENERGY
	std::cout << "bend energy " << C_bend.norm() << std::endl;
	std::cout << "total energy \t\t" << k_stretch * (Cu_stretch.norm() + Cv_stretch.norm()) 
		+ k_shear * C_shear.norm() + k_bend * C_bend.norm() << std::endl;
#endif

	//std::cout << "bend force pair #" << bend_cnt << std::endl;
#endif

	//Edgeidx ehd;
	//for (Mesh::FaceHalfedgeIter fhe_it(mesh_, fl); fhe_it; ++fhe_it) {
	//	Mesh::HalfedgeHandle ohe = mesh_.opposite_halfedge_handle(fhe_it);
	//	Mesh::FaceHandle f = mesh_.face_handle(ohe);
	//	if (f == fr) {
	//		eh_ = mesh_.edge_handle(ohe);
	//	}
	//}
	//getBendForce();

#ifdef USE_GRAVITY
	// add external force
	for (auto iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter)
	{
		GLuint global_index = vertices2indices.at(*iter);
		Eigen::Vector3f gravity(0.0f, - mass_list[global_index] * 9.8f, 0.0f);
		addExternForce(*iter, gravity);
		//break;
	}
#endif

	// after constraints and mass set 
	mass_constrainted = mass_inverse * constraints;
	//std::cout << "mass_constrainted " << (checkSymmetrical(mass_constrainted) ? true : false) << std::endl;
	//std::cout << "constraints " << std::endl << constraints << std::endl;

	//std::cout << "f_total " << std::endl << f_total << std::endl;

}

void BaraffRequire::readPositions()
{
	positions = Eigen::VectorXf(VERTEX_SIZE * 3);
	SurfaceMesh3f* mesh = clothPiece->getMesh();
	for (auto iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter)
	{
		Eigen::Vector3f pos_eigen;
		Veridx vh = *iter;
		copy_v3f(pos_eigen, mesh->point(vh));
		positions.block<3, 1>(vertices2indices.at(vh) * 3, 0) = pos_eigen;
	}
}

void BaraffRequire::writePositions()
{
	SurfaceMesh3f* mesh = clothPiece->getMesh();
	BOOST_FOREACH(Veridx viter, mesh->vertices())
	{
		Point3f pos_cgal;
		Eigen::Vector3f pos_eigen = positions.block<3, 1>(vertices2indices.at(viter) * 3, 0);
		copy_v3f(pos_cgal, pos_eigen);
		mesh->point(viter) = pos_cgal;
	}
	/* after changing position
	* update normals for consistence
	*/
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals =
		mesh->property_map<Faceidx, Vec3f>(clothPiece->pname_faceNormals).first;
	//CGAL::Polygon_mesh_processing::compute_face_normals(mesh, faceNormals);
	SurfaceMesh3f::Property_map<Veridx, Vec3f> vertexNormals =
		mesh->property_map<Veridx, Vec3f>(clothPiece->pname_vertexNormals).first;
	//CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vertexNormals);
	CGAL::Polygon_mesh_processing::compute_normals(*mesh, vertexNormals, faceNormals,
		CGAL::Polygon_mesh_processing::parameters::vertex_point_map(mesh->points()).geom_traits(Kernelf()));

}

/* called once, used for allocating memory */
void BaraffRequire::initial()
{
	SurfaceMesh3f* mesh = clothPiece->getMesh();
	VERTEX_SIZE = mesh->number_of_vertices();
	FACE_SIZE = mesh->number_of_faces();
	EDGE_SIZE = mesh->number_of_edges();

	// initial global indices
	int index = 0;
	for (auto iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter, ++index)
	{
		vertices2indices[*iter] = index;
	}
	index = 0;
	for (auto iter = mesh->faces_begin(); iter != mesh->faces_end(); ++iter, ++index)
	{
		faces2indices[*iter] = index;
	}
	index = 0;
	for (auto iter = mesh->edges_begin(); iter != mesh->edges_end(); ++iter, ++index)
	{
		edges2indices[*iter] = index;
	}

	// initial mass
	for (size_t _i = 0; _i < VERTEX_SIZE; ++_i)
	{
		mass_list.push_back(density);
	}

	readPositions();

	coeff_list.reserve(VERTEX_SIZE * 3);

	reserve_sparsematrix = Eigen::VectorXi::Constant(VERTEX_SIZE * 3, 3 * 10);

	// ------------ allocate memory ------------ 
	df_dx_internal_total = Eigen::SparseMatrix<float>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	df_dx_damp_total = Eigen::SparseMatrix<float>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	df_dv_damp_total = Eigen::SparseMatrix<float>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);

	/* WARNING : preallocate enough memory and inner alignment
	 * should not use setZero(), which will remove the inner alignment
	 * see http://eigen.tuxfamily.org/dox/group__TutorialSparse.html 
	 */
	df_dx_internal_total.reserve(reserve_sparsematrix);
	df_dx_damp_total.reserve(reserve_sparsematrix);
	df_dv_damp_total.reserve(reserve_sparsematrix);

	f_total = Eigen::VectorXf(VERTEX_SIZE * 3);
	v_total = Eigen::VectorXf(VERTEX_SIZE * 3);
	Cu_stretch = Eigen::VectorXf(FACE_SIZE);
	Cv_stretch = Eigen::VectorXf(FACE_SIZE);
	C_shear = Eigen::VectorXf(FACE_SIZE);
	C_bend = Eigen::VectorXf(EDGE_SIZE);
	constraints = Eigen::SparseMatrix<float>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	mass_inverse = Eigen::SparseMatrix<float>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	mass = Eigen::SparseMatrix<float>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	identity = Eigen::SparseMatrix<float>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	
	// ------------ initial constant variables ------------ 
	// initial mass matrix, should not be modified
	coeff_list.clear();
	for (size_t _i = 0; _i < VERTEX_SIZE; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		coeff_list.push_back(Tri_float(_i * 3 + _j, _i * 3 + _j, mass_list[_i]));
	mass.setFromTriplets(coeff_list.begin(), coeff_list.end());

	// initial mass_inverse matrix, should not be modified
	coeff_list.clear();
	for (size_t _i = 0; _i < VERTEX_SIZE; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		coeff_list.push_back(Tri_float(_i * 3 + _j, _i * 3 + _j, 1.0f / mass_list[_i]));
	mass_inverse.setFromTriplets(coeff_list.begin(), coeff_list.end());

	// initial identity matrix, should not be modified
	coeff_list.clear();
	for (size_t _i = 0; _i < VERTEX_SIZE * 3; ++_i)
		coeff_list.push_back(Tri_float(_i, _i, 1.0f));
	identity.setFromTriplets(coeff_list.begin(), coeff_list.end());
	
	// initial planar coordinates
	clothPiece->useVTexCoord2DAsVPlanarCoord3f();
	//clothPiece->getVPlanarCoord3f(vph_planarcoord);
	
	reset(true);
}

// called before each iteration
void BaraffRequire::reset(GLboolean first)
{
	SurfaceMesh3f* mesh = clothPiece->getMesh();

	// update face normal for bend condition 
	// TODO need a better solution
	// if already writted, then skip and update directly
	writePositions();

	// reset vectors
	f_total.setZero();
	Cu_stretch.setZero();
	Cv_stretch.setZero();
	C_shear.setZero();
	C_bend.setZero();
	if (first)
	{
		v_total.setZero();
		//std::cout << "initial v_total nonzero " << std::endl << v_total.nonZeros() << std::endl;
		//std::cout << "initial v_total " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;
	}

	/* WARNING : reset derivatives by self minus
	 * should not use setZero(), for it will remove the inner alignment
	 */
	df_dx_internal_total -= df_dx_internal_total;
	df_dx_damp_total -= df_dx_damp_total;
	df_dv_damp_total -= df_dv_damp_total;

		//df_dx_internal_total.reserve(reserve_sparsematrix);
		//df_dx_damp_total.reserve(reserve_sparsematrix);
		//df_dv_damp_total.reserve(reserve_sparsematrix);

	//else
	//{
	//	//df_dv_damp_total.
	//	const Eigen::Matrix3f zero = Eigen::Matrix3f::Zero();
	//	for (size_t _i = 0; _i < VERTEX_SIZE * 3; ++_i) for (size_t _j = 0; _j < VERTEX_SIZE * 3; ++_j)
	//	{
	//	}
	//}

	// reset constraint
	//if (first)
	{
		coeff_list.clear();
		for (size_t _i = 0; _i < VERTEX_SIZE * 3; ++_i)
			coeff_list.push_back(Tri_float(_i, _i, 1.0f));
		constraints.setFromTriplets(coeff_list.begin(), coeff_list.end());
	}
}

// I - p*pT == 0, p should be unit vector
void BaraffRequire::addConstraint(Veridx vhandle, Eigen::Vector3f direction)
{
	GLuint global_index = vertices2indices.at(vhandle);
	Eigen::Vector3f dir_unit = direction.normalized();
	addBlock33(constraints, global_index, global_index, -dir_unit * dir_unit.transpose());
}

void BaraffRequire::addExternForce(Veridx vhandle, Eigen::Vector3f ext_force)
{
	GLuint global_index = vertices2indices.at(vhandle);
	f_total.block<3, 1>(global_index * 3, 0) += ext_force;
}

void BaraffRequire::getStretchAndShearForce(Faceidx fhandle,
	const SurfaceMesh3f::Property_map<Veridx, Point3f> & vprop_planarcoord,
	float k_stretch, float kd_stretch, float k_shear, float kd_shear, float bu, float bv)
{
	//std::cout << "in1 v_total nonzero " << std::endl << v_total.nonZeros() << std::endl;
	//std::cout << "in1 v_total[0] " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;
	SurfaceMesh3f* mesh = clothPiece->getMesh();
	CGAL::Vertex_around_face_circulator<SurfaceMesh3f> cfviter(mesh->halfedge(fhandle), *mesh), done(cfviter);

	/*
	x0-----x1
	|#####/
	|####/
	|###/
	|##/
	|#/
	x2
	*/

	// vhandles of three vertices
	Veridx vhandles[3];
	{
		vhandles[0] = *cfviter++;
		assert(cfviter != done);
		vhandles[1] = *cfviter++;
		assert(cfviter != done);
		vhandles[2] = *cfviter++;
		assert(cfviter == done);

		// planar coordinate
		Eigen::Vector3f t0, t1, t2;
		copy_v3f(t0, vprop_planarcoord[vhandles[0]]);
		copy_v3f(t1, vprop_planarcoord[vhandles[1]]);
		copy_v3f(t2, vprop_planarcoord[vhandles[2]]);
		float e1, e2, e3;
		e1 = (t0 - t1).norm();
		e2 = (t1 - t2).norm();
		e3 = (t2 - t0).norm();
		if (e1 > e2 && e1 > e3)
		{
			shiftVertices(vhandles[0], vhandles[1], vhandles[2]);
		}
		else if (e3 > e1 && e3 > e2)
		{
			shiftVertices(vhandles[0], vhandles[1], vhandles[2]);
			shiftVertices(vhandles[0], vhandles[1], vhandles[2]);
		}
			
	}
	// global indices of three vertices
	GLuint global_indices[3];
	global_indices[0] = vertices2indices.at(vhandles[0]);
	global_indices[1] = vertices2indices.at(vhandles[1]);
	global_indices[2] = vertices2indices.at(vhandles[2]);
#ifdef DEBUG_FORCE
	std::cout << "global indices ";
	for (size_t _i = 0; _i < 3; ++_i)
	{
		std::cout << " " << global_indices[_i];
	}
	std::cout << std::endl;
#endif
	//for (size_t _i = 0; _i < 3; ++_i)
	//{
	//	if (global_indices[_i] > 303)
	//		std::cout << "ERROR:: index error" << std::endl;
	//	
	//}

	GLuint face_index = faces2indices.at(fhandle);
	
	// vertices velocity
#ifdef DEBUG_FORCE
	//std::cout << "v_total " << std::endl << v_total << std::endl;
	//std::cout << "in2 v_total nonzero " << std::endl << v_total.nonZeros() << std::endl;
	//std::cout << "in2 v_total[0] " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;
#endif

	Eigen::Vector3f v[3];
	for (size_t _i = 0; _i < 3; ++_i)
	{
		v[_i] = v_total.block<3, 1>(global_indices[_i] * 3, 0);
#ifdef DEBUG_FORCE
		std::cout << "v[" << global_indices[_i] << "] " << std::endl << v[_i] << std::endl;
#endif
	}
	//std::cout << "in3 v_total[0] " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;

	// planar coordinate
	Eigen::Vector3f u0, u1, u2;
	copy_v3f(u0, vprop_planarcoord[vhandles[0]]);
	copy_v3f(u1, vprop_planarcoord[vhandles[1]]);
	copy_v3f(u2, vprop_planarcoord[vhandles[2]]);

	Eigen::Vector3f du1 = u1 - u0;
	Eigen::Vector3f du2 = u2 - u0;
#ifdef DEBUG_FORCE
	std::cout << "du1 " << std::endl << du1 << std::endl;
	std::cout << "du2 " << std::endl << du2 << std::endl;
#endif

	// area of triangle in planar coordinate
	float area = fabs((du1.cross(du2)).coeff(2)) * 0.5f;
#ifdef DEBUG_FORCE
	std::cout << "area " << std::endl << area << std::endl;
#endif
	// alpha
	float alpha = pow(area, 0.75f);
#ifdef DEBUG_FORCE
	std::cout << "alpha " << std::endl << alpha << std::endl;
#endif

	// world coordinate
	Eigen::Vector3f x0, x1, x2;
	x0 = positions.block<3, 1>(vertices2indices.at(vhandles[0]) * 3, 0);
	x1 = positions.block<3, 1>(vertices2indices.at(vhandles[1]) * 3, 0);
	x2 = positions.block<3, 1>(vertices2indices.at(vhandles[2]) * 3, 0);
#ifdef DEBUG_FORCE
	std::cout << "x[" << global_indices[0] << "]" << std::endl << x0 << std::endl;
	std::cout << "x[" << global_indices[1] << "]" << std::endl << x1 << std::endl;
	std::cout << "x[" << global_indices[2] << "]" << std::endl << x2 << std::endl;
#endif

	// transform between world coordinate and planar coordinate
	Eigen::Vector3f wu, wv;
	//float dom = max(du1(0) * du2(1) - du2(0) * du1(1), 1e-15f);
	float dom_raw = du1(0) * du2(1) - du2(0) * du1(1);
	float dom = (dom_raw >= 0.0f) ? 
		max(du1(0) * du2(1) - du2(0) * du1(1), 1e-40f) :
		min(du1(0) * du2(1) - du2(0) * du1(1), -1e-40f);
#ifdef DEBUG_FORCE
	std::cout << "dom " << dom << std::endl;
#endif
	wu = (du2(1) * (x1 - x0) - du1(1) * (x2 - x0)) / dom;
	wv = -(du2(0) * (x1 - x0) - du1(0) * (x2 - x0)) / dom;
#ifdef DEBUG_FORCE
	std::cout << "wu " << std::endl << wu << std::endl;
	std::cout << "wv " << std::endl << wv << std::endl;
#endif
	float wu_len, wv_len;
	wu_len = max(wu.norm(), 1e-40f);
	wv_len = max(wv.norm(), 1e-40f);
	Eigen::Vector3f wu_unit, wv_unit;
	wu_unit = wu / wu_len;
	wv_unit = wv / wv_len;
#ifdef DEBUG_FORCE
	std::cout << "wu_len " << std::endl << wu_len << std::endl;
	std::cout << "wv_len " << std::endl << wv_len << std::endl;
#endif

	// derivatives dw_dxi, i = 0, 1, 2
	Eigen::Matrix3f dwu_dxi[3], dwv_dxi[3];
	dwu_dxi[0] = ((du1(1) - du2(1)) / dom) * Eigen::Matrix3f::Identity();
	dwu_dxi[1] = ((du2(1)) / dom) * Eigen::Matrix3f::Identity();
	dwu_dxi[2] = ((-du1(1)) / dom) * Eigen::Matrix3f::Identity();
	// >>>>>
	//dwv_dxi[0] = ((du1(0) - du2(0)) / dom) * Eigen::Matrix3f::Identity();
	//dwv_dxi[1] = ((du2(0)) / dom) * Eigen::Matrix3f::Identity();
	//dwv_dxi[2] = ((-du1(0)) / dom) * Eigen::Matrix3f::Identity();
	// -----  !!! modified 201602041506
	dwv_dxi[0] = ((du2(0) - du1(0)) / dom) * Eigen::Matrix3f::Identity();
	dwv_dxi[1] = ((-du2(0)) / dom) * Eigen::Matrix3f::Identity();
	dwv_dxi[2] = ((du1(0)) / dom) * Eigen::Matrix3f::Identity();
	// <<<<<<
#ifdef DEBUG_FORCE
	for (size_t _i = 0; _i < 3; ++_i)
	{
		std::cout << "dwu_dxi[" << global_indices[_i] << "] " << std::endl << dwu_dxi[_i] << std::endl;
		std::cout << "dwv_dxi[" << global_indices[_i] << "] " << std::endl << dwv_dxi[_i] << std::endl;
	}
#endif

	// ----------------- compute stretch force and its damping force -------------------
#ifdef STRETCH_FORCE
	{
		//std::cout << " ----------- stretch ---------- " << std::endl;

		// stretch conditions in u v directions
		float Cu, Cv;
		Cu = alpha * (wu_len - bu);
		Cv = alpha * (wv_len - bv);
		Cu_stretch[face_index] = Cu;
		Cv_stretch[face_index] = Cv;
#ifdef DEBUG_FORCE
		//std::cout << "wu_len " << wu_len << std::endl;
		//std::cout << "bu " << bu << std::endl;
		//std::cout << "wu_len - bu " << wu_len - bu << std::endl;
		//std::cout << "wv_len " << wv_len << std::endl;
		//std::cout << "bv " << bv << std::endl;
		//std::cout << "wv_len - bv " << wv_len - bv << std::endl;
		std::cout << "Cu " << Cu << std::endl;
		std::cout << "Cv " << Cv << std::endl;
#endif
		//if (Cu < 0 || Cv < 0)
		//{
		//	std::cout << "Cu " << Cu << std::endl;
		//	std::cout << "Cv " << Cv << std::endl;
		//}
		// first ordered derivatives dC_dxi, i = 0, 1, 2
		Eigen::Vector3f dCu_dxi[3], dCv_dxi[3];
		for (size_t _i = 0; _i < 3; ++_i)
		{
			dCu_dxi[_i] = alpha * dwu_dxi[_i] * wu_unit;
			dCv_dxi[_i] = alpha * dwv_dxi[_i] * wv_unit;
#ifdef DEBUG_FORCE
			std::cout << "dCu_dxi[" << global_indices[_i] << "] " << std::endl << dCu_dxi[_i] << std::endl;
			std::cout << "dCv_dxi[" << global_indices[_i] << "] " << std::endl << dCv_dxi[_i] << std::endl;
#endif
		}

		// second ordered derivatives d2C_dxidxj, i = 0, 1, 2
		Eigen::Matrix3f d2Cu_dxidxj[3][3], d2Cv_dxidxj[3][3];
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			d2Cu_dxidxj[_i][_j] = (alpha / wu_len) * dwu_dxi[_i] * dwu_dxi[_j]
				* (Eigen::Matrix3f::Identity() - wu_unit * wu_unit.transpose());
			//std::cout << "d2Cu_dxidxj[" << _i << "]" << "[" << _j << "] " << std::endl << d2Cu_dxidxj[_i][_j] << std::endl;
			d2Cv_dxidxj[_i][_j] = (alpha / wv_len) * dwv_dxi[_i] * dwv_dxi[_j]
				* (Eigen::Matrix3f::Identity() - wv_unit * wv_unit.transpose());
			//std::cout << "d2Cv_dxidxj[" << _i << "]" << "[" << _j << "] " << std::endl << d2Cv_dxidxj[_i][_j] << std::endl;
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			std::cout << "stretch d2Cu_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << d2Cu_dxidxj[_i][_j] << std::endl;
			std::cout << "stretch d2Cv_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << d2Cv_dxidxj[_i][_j] << std::endl;
		}
#endif

		// C_dot is dC_dt 
		float Cu_dot = 0.0f, Cv_dot = 0.0f;
		for (size_t _i = 0; _i < 3; ++_i)
		{
			Cu_dot += dCu_dxi[_i].transpose() * v[_i];
			Cv_dot += dCv_dxi[_i].transpose() * v[_i];
		}
#ifdef DEBUG_FORCE
		std::cout << "Cu_dot " << Cu_dot << std::endl;
		std::cout << "Cv_dot " << Cv_dot << std::endl;
#endif
		// compute and update first order derivatives df_dx, df_dv
		Eigen::Matrix3f dfi_dxj[3][3], dfi_dxj_damp[3][3], dfi_dvj_damp[3][3];
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			// stretch force
			dfi_dxj[_i][_j] = -k_stretch * (
				dCu_dxi[_i] * dCu_dxi[_j].transpose() + dCv_dxi[_i] * dCv_dxi[_j].transpose() +
				d2Cu_dxidxj[_i][_j] * Cu + d2Cv_dxidxj[_i][_j] * Cv);
			//std::cout << "dfi_dxj[" << _i << "]" << "[" << _j << "] " << std::endl << dfi_dxj[_i][_j] << std::endl;
			// stretch damp
			dfi_dxj_damp[_i][_j] = -k_stretch * kd_stretch * (
				d2Cu_dxidxj[_i][_j] * Cu_dot + d2Cv_dxidxj[_i][_j] * Cv_dot);
			//std::cout << "dfi_dxj_damp[" << _i << "]" << "[" << _j << "] " << std::endl << dfi_dxj_damp[_i][_j] << std::endl;
			// stretch damp velocity
			dfi_dvj_damp[_i][_j] = -k_stretch * kd_stretch * (
				dCu_dxi[_i] * dCu_dxi[_j].transpose() + dCv_dxi[_i] * dCv_dxi[_j].transpose());
			//std::cout << "dfi_dvj[" << _i << "]" << "[" << _j << "] " << std::endl << dfi_dvj[_i][_j] << std::endl;
			// update to total f and total v
			addBlock33(df_dx_internal_total, global_indices[_j], global_indices[_i], dfi_dxj[_i][_j]);
#if defined(USE_DAMP) && defined(USE_STRETCH_DAMP)
			addBlock33(df_dx_damp_total, global_indices[_j], global_indices[_i], dfi_dxj_damp[_i][_j]);
			addBlock33(df_dv_damp_total, global_indices[_j], global_indices[_i], dfi_dvj_damp[_i][_j]);
#endif
			//std::cout << "df_dx_total[" << global_indices[_j] << "]" << "[" << global_indices[_i] << "] " << std::endl 
			//	<< df_dx_total.block(3 * global_indices[_j], 3 * global_indices[_i], 3, 3) << std::endl;
			//std::cout << "df_dv_total[" << global_indices[_j] << "]" << "[" << global_indices[_i] << "] " << std::endl
			//	<< df_dv_total.block(3 * global_indices[_j], 3 * global_indices[_i], 3, 3) << std::endl;

		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			std::cout << "stretch dfi_dxj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << dfi_dxj[_i][_j] << std::endl;
		}
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			std::cout << "stretch dfi_dvj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << dfi_dvj[_i][_j] << std::endl;
		}
#endif
		//for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = _i; _j < 3; ++_j)
		//{
		//	std::cout << "stretch dfi_dxj_delta[" << _i << "][" << _j << "] "
		//		<< (checkSymmetrical(dfi_dxj[_i][_j] + dfi_dxj_damp[_i][_j] + dfi_dxj[_j][_i] + dfi_dxj_damp[_j][_i]) ? true : false) << std::endl;
		//	std::cout << "stretch dfi_dvj[" << _i << "][" << _j << "] "
		//		<< (checkSymmetrical(dfi_dvj[_i][_j] + dfi_dvj[_j][_i]) ? true : false) << std::endl;

		//}

		//std::cout << "stretch df_dx_total " << (checkSymmetrical(df_dx_total) ? true : false) << std::endl;
		//std::cout << "stretch df_dv_total " << (checkSymmetrical(df_dv_total) ? true : false) << std::endl;

		// compute and update total f
		Eigen::Vector3f f[3], f_damp[3];
		for (size_t _i = 0; _i < 3; ++_i)
		{
			f[_i] = -k_stretch * (dCu_dxi[_i] * Cu + dCv_dxi[_i] * Cv);
			//std::cout << "f[" << _i << "] " << std::endl << f[_i] << std::endl;
			f_damp[_i] = -k_stretch * kd_stretch * (dCu_dxi[_i] * Cu_dot + dCv_dxi[_i] * Cv_dot);
			//std::cout << "f_damp[" << _i << "] " << std::endl << f_damp[_i] << std::endl;

#ifdef DEBUG_FORCE
			std::cout << "f[" << global_indices[_i] << "] " << std::endl << f[_i] << std::endl;
			std::cout << "f_damp[" << global_indices[_i] << "] " << std::endl << f_damp[_i] << std::endl;
#endif

			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f[_i];
#if defined(USE_DAMP) && defined(USE_STRETCH_DAMP)
			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f_damp[_i];

#endif
			
			//std::cout << "f_total[" << global_indices[_i] << "] " << std::endl << f_total.block<3, 1>(global_indices[_i] * 3, 0) << std::endl;

		}
		//std::cout << std::endl;

	}
#endif
	// ----------------- compute shear force -------------------
#ifdef SHEAR_FORCE
	{
		//std::cout << " ----------- shear ---------- " << std::endl;

		// shear condition
		//float C = alpha * (float(wu.transpose() * wv) - float(du1.transpose() * du2));
		// TODO modified 20160225
		//float C = alpha * wu.transpose() * wv;
		float C = alpha * wu_unit.transpose() * wv_unit;
		C_shear[face_index] = C;
#ifdef DEBUG_FORCE
		std::cout << "C " << C << std::endl;
#endif
		// first ordered derivatives dC_dxi, i = 0, 1, 2
		Eigen::Vector3f dC_dxi[3];
		for (size_t _i = 0; _i < 3; ++_i)
		{
			dC_dxi[_i] = alpha * (dwu_dxi[_i] * wv_unit + dwv_dxi[_i] * wu_unit);
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 3; ++_i)
		{
			std::cout << "shear dC_dxi[" << global_indices[_i] << "] " << std::endl
				<< dC_dxi[_i] << std::endl;
		}
#endif
		// second order derivatives d2C_dxidxj
		Eigen::Matrix3f d2C_dxidxj[3][3];
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			for (size_t _s = 0; _s < 3; ++_s) for (size_t _t = 0; _t < 3; ++_t)
			{
				float temp1 = dwu_dxi[_i].row(_s) * dwv_dxi[_j].row(_t).transpose();
				float temp2 = dwu_dxi[_j].row(_t) * dwv_dxi[_i].row(_s).transpose();
				d2C_dxidxj[_i][_j].coeffRef(_s, _t) = alpha * (temp1 + temp2);
			}
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			std::cout << "shear d2C_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl
				<< d2C_dxidxj[_i][_j] << std::endl;
			//std::cout << "shear d2C_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
			//	<< (checkIdentical(d2C_dxidxj[_i][_j], d2C_dxidxj[_j][_i].transpose()) ? true : false) << std::endl;
		}
#endif

		// C_dot is dC_dt 
		float C_dot = 0.0f;
		for (size_t _i = 0; _i < 3; ++_i)
		{
			C_dot += dC_dxi[_i].transpose() * v[_i];
		}
#ifdef DEBUG_FORCE
		std::cout << "shear C_dot " << C_dot << std::endl;
#endif

		// compute and update first order derivatives df_dx, df_dv
		Eigen::Matrix3f dfi_dxj[3][3], dfi_dxj_damp[3][3], dfi_dvj_damp[3][3];
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			dfi_dxj[_i][_j] = -k_shear * (dC_dxi[_i] * dC_dxi[_j].transpose() + d2C_dxidxj[_i][_j] * C);
			dfi_dxj_damp[_i][_j] = -k_shear * kd_shear * d2C_dxidxj[_i][_j] * C_dot;
			dfi_dvj_damp[_i][_j] = -k_shear * kd_shear * (dC_dxi[_i] * dC_dxi[_j].transpose());

			//if ((_i == 0 && _j == 1) || (_i == 1 && _j == 0))
			//{
			//	std::cout << "dC_dx " << std::endl << dC_dxi[_i] << std::endl;
			//	std::cout << "dfi_dxj " << std::endl << dfi_dvj[_i][_j] << std::endl;
			//}

			// update total data
			addBlock33(df_dx_internal_total, global_indices[_j], global_indices[_i], dfi_dxj[_i][_j]);
#if defined(USE_DAMP) && defined(USE_SHEAR_DAMP)
			addBlock33(df_dx_damp_total, global_indices[_j], global_indices[_i], dfi_dxj_damp[_i][_j]);
			addBlock33(df_dv_damp_total, global_indices[_j], global_indices[_i], dfi_dvj_damp[_i][_j]);
#endif

		}
		//std::cout << "shear df_dv_total " << (checkSymmetrical(df_dv_total) ? true : false) << std::endl;

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			std::cout << "shear dfi_dxj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << dfi_dxj[_i][_j] << std::endl;
		}
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			std::cout << "shear dfi_dvj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << dfi_dvj[_i][_j] << std::endl;
		}
		//std::cout << "shear df_dx_total " << (checkSymmetrical(df_dx_total) ? true : false) << std::endl;
#endif
		//for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		//{
		//	std::cout << "shear (dC_dxi * dC_dxi^T)[" << global_indices[_i] << "][" << global_indices[_j] << "] "
		//		<< (checkIdentical(dC_dxi[_i] * dC_dxi[_j].transpose(), (dC_dxi[_j] * dC_dxi[_i].transpose()).transpose()) ? true : false) << std::endl;
		//}
		//for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		//{
		//	std::cout << "shear dfi_dvj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
		//		<< (checkIdentical(dfi_dvj[_i][_j], dfi_dvj[_j][_i].transpose()) ? true : false) << std::endl;
		//}

		//std::cout << "shear df_dx_total " << (checkSymmetrical(df_dx_total) ? true : false) << std::endl;
		//std::cout << "shear df_dv_total " << (checkSymmetrical(df_dv_total) ? true : false) << std::endl;

		// compute and update total f
		Eigen::Vector3f f[3], f_damp[3];
		for (size_t _i = 0; _i < 3; ++_i)
		{
			f[_i] = -k_shear * dC_dxi[_i] * C;
			f_damp[_i] = -k_shear * kd_shear * dC_dxi[_i] * C_dot;
            
#ifdef DEBUG_FORCE
			std::cout << "f[" << global_indices[_i] << "] " << std::endl << f[_i] << std::endl;
			std::cout << "f_damp[" << global_indices[_i] << "] " << std::endl << f_damp[_i] << std::endl;
#endif

			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f[_i];
#if defined(USE_DAMP) && defined(USE_SHEAR_DAMP)
			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f_damp[_i];
#endif
			//std::cout << "f_total[" << global_indices[_i] << "] " << std::endl << f_total.block<3, 1>(global_indices[_i] * 3, 0) << std::endl;
		}

	}
#endif
}

//#ifdef BEND_FORCE
//
//// TODO : to be moved to basicoperations.cpp
//
//Eigen::Vector3f get_vector3f_block(Tensor3f & tensor, unsigned int block_i, unsigned int block_j)
//{
//	Eigen::Vector3f vec;
//	for (size_t _i = 0; _i < 3; ++_i)
//	{
//		vec[_i] = tensor.coeff(index3(block_i, block_j, _i));
//	}
//	return vec;
//}
//#endif

void BaraffRequire::getBendForce(Faceidx fhandle0, Faceidx fhandle1,
	Edgeidx ehandle, float k_bend, float kd_bend)
{
	SurfaceMesh3f* mesh = clothPiece->getMesh();

	// vhandles of four vertices
	/*
	x0-----x1
	|##A##/#|
	|####/##|
	|###e###|
	|##/##B#|
	|#/#####|
	x2------x3
	*/
	Veridx vhandles[4];

	vhandles[1] = mesh->target(mesh->halfedge(ehandle, 0));
	vhandles[2] = mesh->target(mesh->halfedge(ehandle, 1));

	BOOST_FOREACH(Veridx vh, vertices_around_face(mesh->halfedge(fhandle0), *mesh))
	{
		if (vh == vhandles[1] || vh == vhandles[2]) continue;
		vhandles[0] = vh;
		break;
	}
	BOOST_FOREACH(Veridx vh, vertices_around_face(mesh->halfedge(fhandle1), *mesh))
	{
		if (vh == vhandles[1] || vh == vhandles[2]) continue;
		vhandles[3] = vh;
		break;
	}

	// global indices of three vertices
	GLuint global_indices[4];
	global_indices[0] = vertices2indices.at(vhandles[0]);
	global_indices[1] = vertices2indices.at(vhandles[1]);
	global_indices[2] = vertices2indices.at(vhandles[2]);
	global_indices[3] = vertices2indices.at(vhandles[3]);
#ifdef DEBUG_FORCE
	std::cout << "global indices ";
	for (size_t _i = 0; _i < 4; ++_i)
	{
		std::cout << " " << global_indices[_i];
	}
	std::cout << std::endl;
#endif

	// vertices velocity
	Eigen::Vector3f v[4];
	for (size_t _i = 0; _i < 4; ++_i)
	{
		v[_i] = v_total.block<3, 1>(global_indices[_i] * 3, 0);
#ifdef DEBUG_FORCE
		std::cout << "v[" << global_indices[_i] << "] " << std::endl << v[_i] << std::endl;
#endif
	}
	
	// world coordinate
	Eigen::Vector3f x0, x1, x2, x3;
	x0 = positions.block<3, 1>(vertices2indices.at(vhandles[0]) * 3, 0);
	x1 = positions.block<3, 1>(vertices2indices.at(vhandles[1]) * 3, 0);
	x2 = positions.block<3, 1>(vertices2indices.at(vhandles[2]) * 3, 0);
	x3 = positions.block<3, 1>(vertices2indices.at(vhandles[3]) * 3, 0);
#ifdef DEBUG_FORCE
	std::cout << "x[" << global_indices[0] << "]" << std::endl << x0 << std::endl;
	std::cout << "x[" << global_indices[1] << "]" << std::endl << x1 << std::endl;
	std::cout << "x[" << global_indices[2] << "]" << std::endl << x2 << std::endl;
	std::cout << "x[" << global_indices[3] << "]" << std::endl << x3 << std::endl;
#endif

	GLuint faceA_index = faces2indices.at(fhandle0);
	GLuint faceB_index = faces2indices.at(fhandle1);
	
	// normal of two faces
	Eigen::Vector3f normal_A, normal_B, edge;
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> fprop_normal =
		mesh->property_map<Faceidx, Vec3f>(clothPiece->pname_faceNormals).first;
	// <<<<<<
	copy_v3f(normal_A, fprop_normal[fhandle0]);
	copy_v3f(normal_B, fprop_normal[fhandle1]);
	// ---- TODO use this normal works ?? check reasons
	//normal_A = - (x2 - x1).cross(x0 - x2);
	//normal_B = (x2 - x3).cross(x3 - x1);
	// >>>>>>
	edge = x2 - x1;

	// check whether a triangle shrinks to a line
	float tolerance = 1e-5f;
	if ((x0 - x1).norm() < tolerance || (x0 - x2).norm() < tolerance
		|| (x2 - x1).norm() < tolerance || (x3 - x1).norm() < tolerance
		|| (x3 - x2).norm() < tolerance)
	{
		return;
	}

	//Eigen::Vector3f down(0.0f, -1.0f, 0.0f);
	//if (normal_A.transpose() * down > 0)
	//{
	//	normal_A *= -1;
	//	//std::cout << "normal down " << std::endl;
	//}
	//if (normal_B.transpose() * down > 0)
	//{
	//	normal_B *= -1;
	//	//std::cout << "normal down " << std::endl;
	//}

	float normal_A_len, normal_B_len, edge_len;
	normal_A_len = max(normal_A.norm(), 1e-40f);
	normal_B_len = max(normal_B.norm(), 1e-40f);
	edge_len = max(edge.norm(), 1e-40f);

	Eigen::Vector3f normal_A_unit, normal_B_unit, edge_unit;
	normal_A_unit = normal_A / normal_A_len;
	normal_B_unit = normal_B / normal_B_len;
	edge_unit = edge / edge_len;

#ifdef DEBUG_FORCE
	std::cout << "normal_A len " << normal_A_len << std::endl << normal_A << std::endl;
	std::cout << "normal_B len " << normal_B_len << std::endl << normal_B << std::endl;
	std::cout << "edge len " << edge_len << std::endl << edge << std::endl;
	std::cout << "normal_A unit " << std::endl << normal_A_unit << std::endl;
	std::cout << "normal_B unit " << std::endl << normal_B_unit << std::endl;
	std::cout << "edge unit " << std::endl << edge_unit << std::endl;
#endif

	float cos_theta = normal_A_unit.transpose() * normal_B_unit;
	float sin_theta = normal_A_unit.cross(normal_B_unit).transpose() * edge_unit;

#ifdef DEBUG_FORCE
	std::cout << "sin " << sin_theta << std::endl;
	std::cout << "cos " << cos_theta << std::endl;
#endif

	Eigen::Vector3f qA[4], qB[4], qE[4];
	qA[0] = x2 - x1;
	qA[1] = x0 - x2;
	qA[2] = x1 - x0;
	qA[3] = Eigen::Vector3f::Zero();

	qB[0] = Eigen::Vector3f::Zero(); 
	qB[1] = x2 - x3;
	qB[2] = x3 - x1;
	qB[3] = x1 - x2;

	qE[0] = Eigen::Vector3f::Zero();
	qE[1] = Eigen::Vector3f::Ones();
	qE[2] = -Eigen::Vector3f::Ones();
	qE[3] = Eigen::Vector3f::Zero();

	static const float coeff_qA_xi[4][4] = {
		{ 0, -1, 1, 0 },
		{ 1, 0, -1, 0 },
		{ -1, 1, 0, 0 },
		{ 0, 0, 0, 0 }
	};
	static const float coeff_qB_xi[4][4] = {
		{ 0, 0, 0, 0 },
		{ 0, 0, 1, -1 },
		{ 0, -1, 0, 1 },
		{ 0, 1, -1, 0 }
	};

	Eigen::Matrix3f dqAi_dxj[4][4], dqBi_dxj[4][4], dqEi_dxj[4][4];
	for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
	{
		dqAi_dxj[_i][_j] = coeff_qA_xi[_i][_j]/* / normal_A_len*/ * Eigen::Matrix3f::Identity();
		dqBi_dxj[_i][_j] = coeff_qB_xi[_i][_j]/* / normal_B_len*/ * Eigen::Matrix3f::Identity();
		dqEi_dxj[_i][_j] = Eigen::Matrix3f::Zero();
	}
#ifdef DEBUG_FORCE
	for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
	{
		std::cout << "dqAi_dxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << dqAi_dxj[_i][_j] << std::endl;
		std::cout << "dqBi_dxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << dqBi_dxj[_i][_j] << std::endl;
		std::cout << "dqEi_dxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << dqEi_dxj[_i][_j] << std::endl;
	}
#endif


	// ----------------- compute bend force and its damping force -------------------
#ifdef BEND_FORCE
	{
		// bend condition
		float C = atan2(sin_theta, cos_theta);
		C_bend[edges2indices.at(ehandle)] = C;
#ifdef DEBUG_FORCE
		std::cout << "bend condition " << C << std::endl;
#endif
		//std::cout << "bend sin " << sin_theta << " cos " << cos_theta << " condition " << C << std::endl;

		Eigen::Matrix3f dnA_unit_dxi[4], dnB_unit_dxi[4], de_unit_dxi[4];
		for (size_t _i = 0; _i < 4; ++_i)
		{
			dnA_unit_dxi[_i] = get_S_m3f(qA[_i]) / normal_A_len;
			dnB_unit_dxi[_i] = get_S_m3f(qB[_i]) / normal_B_len;
			de_unit_dxi[_i] = get_S_m3f(qE[_i]) / edge_len;
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i)
		{
			std::cout << "dnA_unit_dxi[" << global_indices[_i] << "] " << std::endl << dnA_unit_dxi[_i] << std::endl;
			std::cout << "dnB_unit_dxi[" << global_indices[_i] << "] " << std::endl << dnB_unit_dxi[_i] << std::endl;
			std::cout << "de_unit_dxi[" << global_indices[_i] << "] " << std::endl << de_unit_dxi[_i] << std::endl;
		}
#endif

		// derivetives of normal vector A/B/e w.r.t x
		Tensor3f d2nA_unit_dxidxj[4][4], d2nB_unit_dxidxj[4][4], d2e_unit_dxidxj[4][4];
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			d2e_unit_dxidxj[_i][_j] = Tensor3f(3, 3, 3);
			d2e_unit_dxidxj[_i][_j].setZero();

			d2nA_unit_dxidxj[_i][_j] = Tensor3f(3, 3, 3);
			d2nB_unit_dxidxj[_i][_j] = Tensor3f(3, 3, 3);
			for (size_t _s = 0; _s < 3; ++_s) for (size_t _t = 0; _t < 3; ++_t)
			{
				Eigen::Vector3f temp1 = 
					get_S_m3f(Eigen::Vector3f(dqAi_dxj[_i][_j].row(_t).transpose()))
					.row(_s).transpose() / normal_A_len;
#ifdef DEBUG_FORCE
				//std::cout << "d2normalA_dxidxj items " << std::endl << temp1 << std::endl;
#endif
				for (size_t _k = 0; _k < 3; ++_k)
				{
					d2nA_unit_dxidxj[_i][_j].coeffRef(index3(_s, _t, _k)) = temp1[_k];
				}

				Eigen::Vector3f temp2 = 
					get_S_m3f(Eigen::Vector3f(dqBi_dxj[_i][_j].row(_t).transpose()))
					.row(_s).transpose() / normal_B_len;
#ifdef DEBUG_FORCE
				//std::cout << "d2normalB_dxidxj items " << std::endl << temp1 << std::endl;
#endif
				for (size_t _k = 0; _k < 3; ++_k)
				{
					d2nB_unit_dxidxj[_i][_j].coeffRef(index3(_s, _t, _k)) = temp2[_k];
				}
			}
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			std::cout << "d2nA_unit_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << d2nA_unit_dxidxj[_i][_j] << std::endl;
			std::cout << "d2nB_unit_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << d2nB_unit_dxidxj[_i][_j] << std::endl;
			std::cout << "d2e_unit_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << d2e_unit_dxidxj[_i][_j] << std::endl;
		}
#endif
		
		Eigen::Vector3f dcos_dxi[4];
		for (size_t _i = 0; _i < 4; ++_i)
		{
			for (size_t _s = 0; _s < 3; ++_s)
			{
				float temp1 = dnA_unit_dxi[_i].row(_s) * normal_B_unit;
				float temp2 = normal_A_unit.transpose() * dnB_unit_dxi[_i].row(_s).transpose();
				dcos_dxi[_i][_s] = temp1 + temp2;
			}
			//dcos_dxi[_i] = dnormalA_dxi[_i] * normal_B_unit + dnormalB_dxi[_i] * normal_A_unit;
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i)
		{
			std::cout << "dcos_dxi[" << global_indices[_i] << "] " << std::endl << dcos_dxi[_i] << std::endl;
		}
#endif

		Eigen::Vector3f dsin_dxi[4];
		for (size_t _i = 0; _i < 4; ++_i)
		{
			for (size_t _s = 0; _s < 3; ++_s)
			{
				//Eigen::Vector3f temp3 = (dnormalA_dxi[_i].row(_s).cross(normal_B_unit)).transpose();
				//Eigen::Vector3f temp4 = dnormalB_dxi[_i].row(_s);
				//Eigen::Vector3f temp5 = normal_A_unit.cross(dnormalB_dxi[_i].row(_s).transpose());
				float temp1 = (dnA_unit_dxi[_i].row(_s).cross(normal_B_unit)
						+ normal_A_unit.cross(dnB_unit_dxi[_i].row(_s)).transpose())
					* edge_unit;
				float temp2 = (normal_A_unit.cross(normal_B_unit)).transpose() 
					* de_unit_dxi[_i].row(_s).transpose();
				dsin_dxi[_i][_s] = temp1 + temp2;
#ifdef DEBUG_FORCE
				//std::cout << "dsin_dxi[" << global_indices[_i] << "] dim [" << _s << "] " << temp1 << " " << temp2 << std::endl;
				//std::cout << "temp3 " << std::endl << temp3 << std::endl;
				//std::cout << "temp4 " << std::endl << temp4 << std::endl;
				//std::cout << "temp5 " << std::endl << temp5 << std::endl;
#endif
			}
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i)
		{
			std::cout << "dsin_dxi[" << global_indices[_i] << "] " << std::endl << dsin_dxi[_i] << std::endl;
		}
#endif

		Eigen::Matrix3f d2cos_dxidxj[4][4];
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			for (size_t _s = 0; _s < 3; ++_s) for (size_t _t = 0; _t < 3; ++_t)
			{
				float temp1 = get_vector3f_block(d2nA_unit_dxidxj[_i][_j], _s, _t).transpose() * normal_B_unit;
				float temp2 = dnB_unit_dxi[_j].row(_t) * dnA_unit_dxi[_i].row(_s).transpose();
				float temp3 = dnA_unit_dxi[_j].row(_t) * dnB_unit_dxi[_i].row(_s).transpose();
				float temp4 = normal_A_unit.transpose() * get_vector3f_block(d2nB_unit_dxidxj[_i][_j], _s, _t);
				d2cos_dxidxj[_i][_j].coeffRef(_s, _t) = temp1 + temp2 + temp3 + temp4;
#ifdef DEBUG_FORCE
				//std::cout << "d2cos_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] dim " 
				//	<< _s << " " << _t << " items " << temp1 << " " << temp2 << " " << temp3 << " " << temp4 << std::endl;
#endif
			}
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			std::cout << "d2cos_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << d2cos_dxidxj[_i][_j] << std::endl;
		}
#endif

		Eigen::Matrix3f d2sin_dxidxj[4][4];
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			for (size_t _s = 0; _s < 3; ++_s) for (size_t _t = 0; _t < 3; ++_t)
			{
				float temp1 = 
					(get_vector3f_block(d2nA_unit_dxidxj[_i][_j], _s, _t).transpose().cross(normal_B_unit)
						+ dnA_unit_dxi[_i].row(_s).cross(dnB_unit_dxi[_j].row(_t))
						+ dnA_unit_dxi[_j].row(_t).cross(dnB_unit_dxi[_i].row(_s))
						+ normal_A_unit.cross(get_vector3f_block(d2nB_unit_dxidxj[_i][_j], _s, _t)).transpose())
					* edge_unit;
				float temp2 =
					(dnA_unit_dxi[_i].row(_s).cross(normal_B_unit)
						+ normal_A_unit.cross(dnB_unit_dxi[_i].row(_s)).transpose())
					* de_unit_dxi[_j].row(_t).transpose();
				float temp3 =
					(dnA_unit_dxi[_j].row(_t).cross(normal_B_unit)
						+ normal_A_unit.cross(dnB_unit_dxi[_j].row(_t)).transpose())
					* de_unit_dxi[_i].row(_s).transpose();
				float temp4 =
					normal_A_unit.cross(normal_B_unit).transpose()
					* get_vector3f_block(d2e_unit_dxidxj[_i][_j], _s, _t);
				d2sin_dxidxj[_i][_j].coeffRef(_s, _t) = temp1 + temp2 + temp3 + temp4;

#ifdef DEBUG_FORCE
				//std::cout << "d2sin_dxidxj items " << temp1 << " " << temp2 << " " << temp3 << " " << temp4 << std::endl;
#endif
			}
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			std::cout << "d2sin_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << d2sin_dxidxj[_i][_j] << std::endl;
		}
#endif

		Eigen::Vector3f dC_dxi[4];
		for (size_t _i = 0; _i < 4; ++_i)
		{
			dC_dxi[_i] = cos_theta * dsin_dxi[_i] - sin_theta * dcos_dxi[_i];
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i)
		{
			std::cout << "dC_dxi[" << global_indices[_i] << "] " << std::endl << dC_dxi[_i] << std::endl;
		}
#endif

		Eigen::Matrix3f d2C_dxidxj[4][4];
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			for (size_t _s = 0; _s < 3; ++_s) for (size_t _t = 0; _t < 3; ++_t)
			{
				float temp1 = cos_theta * d2sin_dxidxj[_i][_j].coeff(_s, _t)
					- sin_theta * d2cos_dxidxj[_i][_j].coeff(_s, _t);
				float temp2 = dcos_dxi[_j][_t] * dsin_dxi[_i][_s] 
					- dsin_dxi[_j][_t] * dcos_dxi[_i][_s];
				//float temp1 = cos_theta * d2sin_dxidxj[_i][_j].coeff(_s, _t)
				//	- sin_theta * d2cos_dxidxj[_i][_j].coeff(_s, _t);
				//float temp2 = (sin_theta * sin_theta - cos_theta * cos_theta) *
				//	(dsin_dxi[_i][_s] * dcos_dxi[_j][_t] + dcos_dxi[_i][_s] * dsin_dxi[_j][_t]);
				//float temp3 = 2 * sin_theta * cos_theta *
				//	(dcos_dxi[_i][_s] * dcos_dxi[_j][_t] - dsin_dxi[_i][_s] * dsin_dxi[_j][_t]);
				d2C_dxidxj[_i][_j].coeffRef(_s, _t) = temp1 + temp2;
#ifdef DEBUG_FORCE
				//std::cout << "d2C_dxidxj items " << temp1 << " " << temp2 << " " << temp3 << std::endl;
#endif
			}
		}

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			std::cout << "d2C_dxidxj[" << global_indices[_i] << "][" << global_indices[_j] << "] " << std::endl << d2C_dxidxj[_i][_j] << std::endl;
		}
#endif

		// C_dot is dC_dt 
		float C_dot = 0.0f;
		for (size_t _i = 0; _i < 4; ++_i)
		{
			C_dot += dC_dxi[_i].transpose() * v[_i];
		}

#ifdef DEBUG_FORCE
		std::cout << "bend C_dot " << C_dot << std::endl;
#endif

		// compute and update first order derivatives df_dx, df_dv
		Eigen::Matrix3f dfi_dxj[4][4], dfi_dxj_damp[4][4], dfi_dvj_damp[4][4];
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			dfi_dxj[_i][_j] = -k_bend * (dC_dxi[_i] * dC_dxi[_j].transpose() + d2C_dxidxj[_i][_j] * C);
			dfi_dxj_damp[_i][_j] = -k_bend * kd_bend * d2C_dxidxj[_i][_j] * C_dot;
			dfi_dvj_damp[_i][_j] = -k_bend * kd_bend * (dC_dxi[_i] * dC_dxi[_j].transpose());

			//if ((_i == 0 && _j == 1) || (_i == 1 && _j == 0))
			//{
			//	std::cout << "dC_dx " << std::endl << dC_dxi[_i] << std::endl;
			//	std::cout << "dfi_dxj " << std::endl << dfi_dvj[_i][_j] << std::endl;
			//}

			// update total data
			addBlock33(df_dx_internal_total, global_indices[_j], global_indices[_i], dfi_dxj[_i][_j]);
#if defined(USE_DAMP) && defined(USE_BEND_DAMP)
			addBlock33(df_dx_damp_total, global_indices[_j], global_indices[_i], dfi_dxj_damp[_i][_j]);
			addBlock33(df_dv_damp_total, global_indices[_j], global_indices[_i], dfi_dvj_damp[_i][_j]);
#endif

		}
		//std::cout << "bend df_dv_total " << (checkSymmetrical(df_dv_total) ? true : false) << std::endl;

#ifdef DEBUG_FORCE
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			std::cout << "bend dfi_dxj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << dfi_dxj[_i][_j] << std::endl;
		}
		for (size_t _i = 0; _i < 4; ++_i) for (size_t _j = 0; _j < 4; ++_j)
		{
			std::cout << "bend dfi_dvj[" << global_indices[_i] << "][" << global_indices[_j] << "] "
				<< std::endl << dfi_dvj[_i][_j] << std::endl;
		}
		//std::cout << "bend df_dx_total " << (checkSymmetrical(df_dx_total) ? true : false) << std::endl;
#endif

		//std::cout << "bend df_dx_total " << (checkSymmetrical(df_dx_total) ? true : false) << std::endl;
		//std::cout << "bend df_dv_total " << (checkSymmetrical(df_dv_total) ? true : false) << std::endl;

		// compute and update total f
		Eigen::Vector3f f[4], f_damp[4];
		for (size_t _i = 0; _i < 4; ++_i)
		{
			f[_i] = -k_bend * dC_dxi[_i] * C;
			f_damp[_i] = -k_bend * kd_bend * dC_dxi[_i] * C_dot;

#ifdef DEBUG_FORCE
			std::cout << "f[" << global_indices[_i] << "] " << std::endl << f[_i] << std::endl;
			std::cout << "f_damp[" << global_indices[_i] << "] " << std::endl << f_damp[_i] << std::endl;
#endif

			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f[_i];
#if defined(USE_DAMP) && defined(USE_BEND_DAMP)
			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f_damp[_i];
		#endif
			//std::cout << "f_total[" << global_indices[_i] << "] " << std::endl << f_total.block<3, 1>(global_indices[_i] * 3, 0) << std::endl;
		}
		
		//std::cout << "bend force end" << std::endl;

	}
#endif
}

void BaraffRequire::update(const Eigen::VectorXf & v_delta)
{
	positions += time_step * (v_total + v_delta);
	v_total += v_delta;
	//std::cout << "delta position " << std::endl << v_total << std::endl;
}

void BaraffRequire::writeMesh()
{
	writePositions();
}

void BaraffRequire::exportShearConditionData(float* & dataBuffer, GLuint & dataSize)
{
	SurfaceMesh3f* mesh = clothPiece->getMesh();
	float max = C_shear.maxCoeff(), min = C_shear.minCoeff();
	//std::cout << "shear condition range " << min << ", " << max << std::endl;
	Eigen::VectorXf C_shear_normalized(mesh->number_of_faces());
	C_shear_normalized.setOnes();
	C_shear_normalized = (C_shear - C_shear_normalized * min) / (max - min);
	dataSize = mesh->number_of_vertices();
	dataBuffer = new float[dataSize];
	memset(dataBuffer, sizeof(float) * dataSize, 0);
	// TODO
	//std::cout << "shear condition size " << dataSize << std::endl;
	//for (auto viter = mesh->vertices_begin(); viter != mesh->vertices_end(); ++viter)
	//{
	//	Veridx vhd = *viter;
	//	float condition = 0.0f;
	//	GLuint cnt = 0;
	//	for (auto cvfiter = mesh->cvf_begin(vhd); cvfiter != mesh->cvf_end(vhd); ++cvfiter, ++cnt)
	//	{
	//		condition += C_shear_normalized[faces2indices.at(*cvfiter)];
	//	}
	//	dataBuffer[vertices2indices.at(vhd)] = condition / (float) cnt;
	//}
}

void BaraffRequire::exportBendConditionData(float *& dataBuffer, GLuint & dataSize)
{
	// TODO
	Eigen::VectorXf temp = C_bend;
	Eigen::Index len = temp.size();
	float sum = 0;
	for (size_t _i = 0; _i < len; ++_i)
	{
		sum += fabs(temp[_i]);
	}
	//std::cout << "bend condition norm " << C_bend.norm() << std::endl;
	//std::cout << "bend condition sum " << sum << std::endl;
}

