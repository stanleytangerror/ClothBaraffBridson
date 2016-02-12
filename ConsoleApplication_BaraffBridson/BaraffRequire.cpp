#include "BaraffRequire.h"

#include "BasicOperations.h"

#include <Eigen\Core>

#include <iostream>

Eigen::SparseMatrix<GLfloat> BaraffRequire::exportA()
{
	//Eigen::SparseMatrix<GLfloat> temp1 = mass_constrainted * df_dv_total;
	//std::cout << "mass_constrainted * df_dv_total diagonal" << std::endl << temp1.diagonal() << std::endl;
	//std::cout << "mass_constrainted * df_dv_total " << std::endl << temp1.size() << std::endl;
	//Eigen::SparseMatrix<GLfloat> temp2 = mass_constrainted * df_dx_total;
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

	//Eigen::SparseMatrix<GLfloat> val1 = (mass_constrainted * df_dv_total);
	//std::cout << "val1 " << (checkSymmetrical(val1) ? true : false) << std::endl;
	//Eigen::SparseMatrix<GLfloat> val2 = (mass_constrainted * df_dx_total);
	//std::cout << "val2 " << (checkSymmetrical(val2) ? true : false) << std::endl;
	//Eigen::SparseMatrix<GLfloat> val3 = val1 + time_step * val2;
	//std::cout << "val3 " << (checkSymmetrical(val3) ? true : false) << std::endl;
	//Eigen::SparseMatrix<GLfloat> val4 = (-time_step) * val3 + identity;
	
	Eigen::SparseMatrix<GLfloat> A = identity - time_step * (mass_constrainted * df_dv_total)
		- time_step * time_step * (mass_constrainted * df_dx_total);
	//std::cout << "A " << (checkSymmetrical(A, 1e-6) ? true : false) << std::endl;
	//std::cout << "A diagonal " << std::endl << A.diagonal() << std::endl;
	//std::cout << "A nonzero " << std::endl << A.nonZeros() << std::endl;
	return A;
}

Eigen::SparseMatrix<GLfloat> BaraffRequire::exportS()
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

	
	Eigen::VectorXf b = mass_constrainted * (f_total + df_dx_total * v_total * time_step) * time_step;

	//std::cout << "b " << std::endl << b << std::endl;
	//std::cout << "b " << std::endl << b.size() << std::endl;

	return b;
}

#define USE_GRAVITY

void BaraffRequire::compute(GLfloat time_step)
{
	reset(false);

	this->time_step = time_step;

	PolyArrayMesh* mesh = clothPiece->getMesh();

#ifdef USE_CONSTRAINTS
	// add constraints
	OpenMesh::VertexHandle vh = *(mesh->vertices_begin());
	addConstraint(vh, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	addConstraint(vh, Eigen::Vector3f(0.0f, 1.0f, 1.0f));
	addConstraint(vh, Eigen::Vector3f(1.0f, 0.0f, 1.0f));
	std::cout << "constraints " << (checkSymmetrical(constraints) ? true : false) << std::endl;
#endif

	// forces, velocities and their derivatives
	//std::cout << "f_total without stretch " << std::endl << f_total << std::endl;
	//std::cout << "df_dx_total without stretch " << std::endl << df_dx_total << std::endl;
	//std::cout << "out v_total " << std::endl << v_total << std::endl;
	//std::cout << "out v_total nonzero " << std::endl << v_total.nonZeros() << std::endl;
	//std::cout << "out v_total[0] " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;
	size_t face_cnt = 0;
	for (auto iter = mesh->faces_begin(); iter != mesh->faces_end(); ++iter)
	{
		getStretchAndShearForce(*iter, vph_planarcoord, k_stretch, kd_stretch, k_shear, kd_shear, bu, bv);
		face_cnt += 1;
		//break;
	}
	//std::cout << "C_shear " << std::endl << C_shear << std::endl;
	//std::cout << "C_shear nonzero " << C_shear.nonZeros() << std::endl;
	//std::cout << "C_shear norm " << C_shear.norm() << std::endl;
	//GLfloat magnitude = max(fabs(C_shear.maxCoeff()), fabs(C_shear.minCoeff()));
	//Eigen::VectorXf temp = C_shear / magnitude;
	//std::cout << "C_shear absmax " << magnitude << std::endl;
	//std::cout << "C_shear norm absmax " << max(fabs(temp.maxCoeff()), fabs(temp.minCoeff())) << std::endl;
	//std::cout << "f_total with stretch " << std::endl << f_total << std::endl;
	//std::cout << "f_total with stretch block " << std::endl << f_total.block<30, 1>(0, 0) << std::endl;
	//std::cout << "face_cnt " << std::endl << face_cnt << std::endl;
	//std::cout << "df_dx_total with stretch " << std::endl << df_dx_total << std::endl;

	// TODO add bend force
	//getBendForce();

#ifdef USE_GRAVITY
	// add external force
	for (auto iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter)
	{
		GLuint global_index = vertices2indices[*iter];
		Eigen::Vector3f gravity(0.0f, 0.0f, mass[global_index] * 9.8f);
		addExternForce(*iter, gravity);
		break;
	}
#endif

	// after constraints and mass set 
	mass_constrainted = mass_inverse * constraints;
	//std::cout << "mass_constrainted " << (checkSymmetrical(mass_constrainted) ? true : false) << std::endl;

	//std::cout << "f_total " << std::endl << f_total << std::endl;

}

void BaraffRequire::readPositions()
{
	positions = Eigen::VectorXf(VERTEX_SIZE * 3);
	PolyArrayMesh* mesh = clothPiece->getMesh();
	for (auto iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter)
	{
		Eigen::Vector3f pos_eigen;
		OpenMesh::VertexHandle vh = *iter;
		copy_v3f(pos_eigen, mesh->point(vh));
		positions.block<3, 1>(vertices2indices[vh] * 3, 0) = pos_eigen;
	}
}

void BaraffRequire::writePositions()
{
	PolyArrayMesh* mesh = clothPiece->getMesh();
	for (auto iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter)
	{
		OpenMesh::Vec3f pos_openmesh;
		Eigen::Vector3f pos_eigen = positions.block<3, 1>(vertices2indices[*iter] * 3, 0);
		copy_v3f(pos_openmesh, pos_eigen);
		mesh->point(*iter) = pos_openmesh;
	}
}

/* called once, used for allocating memory */
void BaraffRequire::initial()
{
	PolyArrayMesh* mesh = clothPiece->getMesh();
	VERTEX_SIZE = mesh->n_vertices();
	FACE_SIZE = mesh->n_faces();

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

	// initial mass
	for (size_t _i = 0; _i < VERTEX_SIZE; ++_i)
	{
		mass.push_back(1.0f);
	}

	readPositions();

	coeff_list.reserve(VERTEX_SIZE * 3);

	// ------------ allocate memory ------------ 
	df_dx_total = Eigen::SparseMatrix<GLfloat>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	df_dv_total = Eigen::SparseMatrix<GLfloat>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	f_total = Eigen::VectorXf(VERTEX_SIZE * 3);
	v_total = Eigen::VectorXf(VERTEX_SIZE * 3);
	C_shear = Eigen::VectorXf(FACE_SIZE);
	constraints = Eigen::SparseMatrix<GLfloat>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	mass_inverse = Eigen::SparseMatrix<GLfloat>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	identity = Eigen::SparseMatrix<GLfloat>(VERTEX_SIZE * 3, VERTEX_SIZE * 3);
	
	// ------------ initial constant variables ------------ 
	// initial mass_inverse matrix, should not be modified
	coeff_list.clear();
	for (size_t _i = 0; _i < VERTEX_SIZE; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		coeff_list.push_back(Tri_GLfloat(_i * 3 + _j, _i * 3 + _j, mass[_i]));
	mass_inverse.setFromTriplets(coeff_list.begin(), coeff_list.end());

	// initial identity matrix, should not be modified
	coeff_list.clear();
	for (size_t _i = 0; _i < VERTEX_SIZE * 3; ++_i)
		coeff_list.push_back(Tri_GLfloat(_i, _i, 1.0f));
	identity.setFromTriplets(coeff_list.begin(), coeff_list.end());
	
	// initial planar coordinates
	clothPiece->getVPlanarCoord3f(vph_planarcoord);
	
	reset(true);
}

// called before each iteration
void BaraffRequire::reset(GLboolean first)
{
	PolyArrayMesh* mesh = clothPiece->getMesh();

	// reset vectors
	f_total.setZero();
	C_shear.setZero();
	if (first)
	{
		v_total.setZero();
		//std::cout << "initial v_total nonzero " << std::endl << v_total.nonZeros() << std::endl;
		//std::cout << "initial v_total " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;
	}

	// reset derivatives
	df_dv_total.setZero();
	df_dx_total.setZero();

	// reset constraint
	coeff_list.clear();
	for (size_t _i = 0; _i < VERTEX_SIZE * 3; ++_i)
		coeff_list.push_back(Tri_GLfloat(_i, _i, 1.0f));
	constraints.setFromTriplets(coeff_list.begin(), coeff_list.end());

}

void BaraffRequire::addConstraint(PolyArrayMesh::VertexHandle vhandle, Eigen::Vector3f direction)
{
	GLuint global_index = vertices2indices[vhandle];
	addBlock33(constraints, global_index, global_index, - direction * direction.transpose());
}

void BaraffRequire::addExternForce(PolyArrayMesh::VertexHandle vhandle, Eigen::Vector3f ext_force)
{
	GLuint global_index = vertices2indices[vhandle];
	f_total.block<3, 1>(global_index * 3, 0) += ext_force;
}

#define STRETCH_FORCE
//#define SHEAR_FORCE
//#define USE_DAMP
//#define DEBUG_FORCE

void BaraffRequire::getStretchAndShearForce(PolyArrayMesh::FaceHandle fhandle,
	const OpenMesh::VPropHandleT<OpenMesh::Vec3f> & vprop_planarcoord, 
	GLfloat k_stretch, GLfloat kd_stretch, GLfloat k_shear, GLfloat kd_shear, GLfloat bu, GLfloat bv)
{
	//std::cout << "in1 v_total nonzero " << std::endl << v_total.nonZeros() << std::endl;
	//std::cout << "in1 v_total[0] " << std::endl << v_total.block<3, 1>(0, 0) << std::endl;
	PolyArrayMesh* mesh = clothPiece->getMesh();
	PolyArrayMesh::ConstFaceVertexIter cfviter = mesh->cfv_iter(fhandle);

	// vhandles of three vertices
	PolyArrayMesh::VertexHandle vhandles[3];
	vhandles[0] = *cfviter++;
	vhandles[1] = *cfviter++;
	vhandles[2] = *cfviter;

	// global indices of three vertices
	GLuint global_indices[3];
	global_indices[0] = vertices2indices[vhandles[0]];
	global_indices[1] = vertices2indices[vhandles[1]];
	global_indices[2] = vertices2indices[vhandles[2]];
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

	GLuint face_index = faces2indices[fhandle];
	
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
	copy_v3f(u0, mesh->property(vprop_planarcoord, vhandles[0]));
	copy_v3f(u1, mesh->property(vprop_planarcoord, vhandles[1]));
	copy_v3f(u2, mesh->property(vprop_planarcoord, vhandles[2]));

	Eigen::Vector3f du1 = u1 - u0;
	Eigen::Vector3f du2 = u2 - u0;
#ifdef DEBUG_FORCE
	std::cout << "du1 " << std::endl << du1 << std::endl;
	std::cout << "du2 " << std::endl << du2 << std::endl;
#endif

	// area of triangle in planar coordinate
	GLfloat area = fabs((du1.cross(du2)).coeff(2)) * 0.5f;
#ifdef DEBUG_FORCE
	std::cout << "area " << std::endl << area << std::endl;
#endif
	// alpha
	GLfloat alpha = pow(area, 0.75f);
#ifdef DEBUG_FORCE
	std::cout << "alpha " << std::endl << alpha << std::endl;
#endif

	// world coordinate
	Eigen::Vector3f x0, x1, x2;
	x0 = positions.block<3, 1>(vertices2indices[vhandles[0]] * 3, 0);
	x1 = positions.block<3, 1>(vertices2indices[vhandles[1]] * 3, 0);
	x2 = positions.block<3, 1>(vertices2indices[vhandles[2]] * 3, 0);
#ifdef DEBUG_FORCE
	std::cout << "x[" << global_indices[0] << "]" << std::endl << x0 << std::endl;
	std::cout << "x[" << global_indices[1] << "]" << std::endl << x1 << std::endl;
	std::cout << "x[" << global_indices[2] << "]" << std::endl << x2 << std::endl;
#endif

	// transform between world coordinate and planar coordinate
	Eigen::Vector3f wu, wv;
	GLfloat dom = du1(0) * du2(1) - du2(0) * du1(1);
#ifdef DEBUG_FORCE
	std::cout << "dom " << dom << std::endl;
#endif
	wu = (du2(1) * (x1 - x0) - du1(1) * (x2 - x0)) / dom;
	wv = -(du2(0) * (x1 - x0) - du1(0) * (x2 - x0)) / dom;
#ifdef DEBUG_FORCE
	std::cout << "wu " << std::endl << wu << std::endl;
	std::cout << "wv " << std::endl << wv << std::endl;
#endif
	GLfloat wu_len, wv_len;
	wu_len = max(wu.norm(), 1e-15f);
	wv_len = max(wv.norm(), 1e-15f);
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
		GLfloat Cu, Cv;
		Cu = alpha * (wu_len - bu);
		Cv = alpha * (wv_len - bv);
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
		if (Cu < 0 || Cv < 0)
		{
			std::cout << "Cu " << Cu << std::endl;
			std::cout << "Cv " << Cv << std::endl;
		}
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
		GLfloat Cu_dot = 0.0f, Cv_dot = 0.0f;
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
		Eigen::Matrix3f dfi_dxj[3][3], dfi_dxj_damp[3][3], dfi_dvj[3][3];
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
			dfi_dvj[_i][_j] = -k_stretch * kd_stretch * (
				dCu_dxi[_i] * dCu_dxi[_j].transpose() + dCv_dxi[_i] * dCv_dxi[_j].transpose());
			//std::cout << "dfi_dvj[" << _i << "]" << "[" << _j << "] " << std::endl << dfi_dvj[_i][_j] << std::endl;
			// update to total f and total v
			addBlock33(df_dx_total, global_indices[_j], global_indices[_i], dfi_dxj[_i][_j]);
#ifdef USE_DAMP
			addBlock33(df_dx_total, global_indices[_j], global_indices[_i], dfi_dxj_damp[_i][_j]);
#endif
			addBlock33(df_dv_total, global_indices[_j], global_indices[_i], dfi_dvj[_i][_j]);
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
#ifdef USE_DAMP
			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f_damp[_i];

#endif



			//std::cout << "f_total[" << global_indices[_i] << "] " << std::endl << f_total.block<3, 1>(global_indices[_i] * 3, 0) << std::endl;

		}
		//std::cout << std::endl;

		// velocity not related to stretch force
	}
#endif
	// ----------------- compute shear force -------------------
#ifdef SHEAR_FORCE
	{
		//std::cout << " ----------- shear ---------- " << std::endl;

		// shear condition
		GLfloat C = alpha * wu.transpose() * wv;
		C_shear[face_index] = C;
#ifdef DEBUG_FORCE
		std::cout << "C " << C << std::endl;
#endif
		// first ordered derivatives dC_dxi, i = 0, 1, 2
		Eigen::Vector3f dC_dxi[3];
		for (size_t _i = 0; _i < 3; ++_i)
		{
			dC_dxi[_i] = alpha * (dwu_dxi[_i] * wv + dwv_dxi[_i] * wu);
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
			d2C_dxidxj[_i][_j] = alpha * (dwu_dxi[_i] * dwv_dxi[_j] + dwu_dxi[_j] * dwv_dxi[_i]);
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
		GLfloat C_dot = 0.0f;
		for (size_t _i = 0; _i < 3; ++_i)
		{
			C_dot += dC_dxi[_i].transpose() * v[_i];
		}
#ifdef DEBUG_FORCE
		std::cout << "shear C_dot " << C_dot << std::endl;
#endif

		// compute and update first order derivatives df_dx, df_dv
		Eigen::Matrix3f dfi_dxj[3][3], dfi_dxj_damp[3][3], dfi_dvj[3][3];
		for (size_t _i = 0; _i < 3; ++_i) for (size_t _j = 0; _j < 3; ++_j)
		{
			dfi_dxj[_i][_j] = -k_shear * (dC_dxi[_i] * dC_dxi[_j].transpose() + d2C_dxidxj[_i][_j] * C);
			dfi_dxj_damp[_i][_j] = -k_shear * kd_shear * d2C_dxidxj[_i][_j] * C_dot;
			dfi_dvj[_i][_j] = -k_shear * kd_shear * (dC_dxi[_i] * dC_dxi[_j].transpose());

			//if ((_i == 0 && _j == 1) || (_i == 1 && _j == 0))
			//{
			//	std::cout << "dC_dx " << std::endl << dC_dxi[_i] << std::endl;
			//	std::cout << "dfi_dxj " << std::endl << dfi_dvj[_i][_j] << std::endl;
			//}

			// update total data
			addBlock33(df_dx_total, global_indices[_j], global_indices[_i], dfi_dxj[_i][_j]);
#ifdef USE_DAMP
			addBlock33(df_dx_total, global_indices[_j], global_indices[_i], dfi_dxj_damp[_i][_j]);
#endif
			addBlock33(df_dv_total, global_indices[_j], global_indices[_i], dfi_dvj[_i][_j]);

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
#ifdef USE_DAMP
			f_total.block<3, 1>(global_indices[_i] * 3, 0) += f_damp[_i];
#endif
			//std::cout << "f_total[" << global_indices[_i] << "] " << std::endl << f_total.block<3, 1>(global_indices[_i] * 3, 0) << std::endl;
		}

	}
#endif
}

void BaraffRequire::getBendForce(PolyArrayMesh::FaceHandle fhandle0, PolyArrayMesh::FaceHandle fhandle1,
	PolyArrayMesh::EdgeHandle ehandle, const OpenMesh::VPropHandleT<OpenMesh::Vec3f> & vprop_planarcoord,
	GLfloat k_bend, GLfloat kd_bend)
{
	// TODO 
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

void BaraffRequire::exportConditionData(GLfloat* & dataBuffer, GLuint & dataSize)
{
	PolyArrayMesh* mesh = clothPiece->getMesh();
	GLfloat max = C_shear.maxCoeff(), min = C_shear.minCoeff();
	//std::cout << "shear condition range " << min << ", " << max << std::endl;
	Eigen::VectorXf C_shear_normalized(mesh->n_faces());
	C_shear_normalized.setOnes();
	C_shear_normalized = (C_shear - C_shear_normalized * min) / (max - min);
	dataSize = mesh->n_vertices();
	dataBuffer = new GLfloat[dataSize];
	memset(dataBuffer, sizeof(GLfloat) * dataSize, 0);
	//std::cout << "shear condition size " << dataSize << std::endl;
	for (auto viter = mesh->vertices_begin(); viter != mesh->vertices_end(); ++viter)
	{
		PolyArrayMesh::VertexHandle vhd = *viter;
		GLfloat condition = 0.0f;
		GLuint cnt = 0;
		for (auto cvfiter = mesh->cvf_begin(vhd); cvfiter != mesh->cvf_end(vhd); ++cvfiter, ++cnt)
		{
			condition += C_shear_normalized[faces2indices[*cvfiter]];
		}
		dataBuffer[vertices2indices[vhd]] = condition / (GLfloat) cnt;
	}
}
