#include "ClothPiece.h"
#include <iostream>

#ifdef OPENMESH_BASED

void ClothPiece::import(const Mesh mesh)
{
	/* load vertexes */
	std::map<GLuint, Veridx> vindices2vhandles;
	for (size_t i = 0; i < mesh.vertices.size(); ++i)
	{
		vindices2vhandles[i] = this->PolyMesh->add_vertex(
			PolyArrayMesh::Point(mesh.vertices[i].Position[0], mesh.vertices[i].Position[1], mesh.vertices[i].Position[2]));
	}
	/* set faces */
	std::vector<Veridx> face_vhandles;
	for (GLuint vindex : mesh.indices)
	{
		face_vhandles.push_back(vindices2vhandles[vindex]);
		if (face_vhandles.size() % EDGES == 0)
		{
			this->PolyMesh->add_face(face_vhandles);
			face_vhandles.clear();
		}
	}
	/* load texcoord 2d */
	PolyMesh->request_vertex_texcoords2D();
	if (PolyMesh->has_vertex_texcoords2D())
	for (size_t i = 0; i < mesh.vertices.size(); ++i)
	{
		PolyMesh->set_texcoord2D(vindices2vhandles[i], PolyArrayMesh::TexCoord2D(mesh.vertices[i].TexCoords[0], mesh.vertices[i].TexCoords[1]));
	}
	/* generate vertex normal conditioning on face normal */
	PolyMesh->request_face_normals();
	PolyMesh->update_face_normals();
	PolyMesh->request_vertex_normals();
	PolyMesh->update_vertex_normals();


	std::cout << "INFO::LOAD MESH " << std::endl;
	std::cout << "> #polygon " << EDGES
		<< ", #vertices " << PolyMesh->n_vertices()
		<< ", #edges " << PolyMesh->n_edges()
		<< ", #halfedges " << PolyMesh->n_halfedges()
		<< ", #faces " << PolyMesh->n_faces() << std::endl;
}

/* export data for VBO and EBO for drawing */
void ClothPiece::exportPos3fNorm3fBuffer(
	GLfloat* & vertexBuffer, GLfloat* & vertexNormalBuffer, GLuint & vertexSize, 
	GLuint* & elementBuffer, GLuint & elementSize)
	const
{
	PolyArrayMesh* mesh = this->PolyMesh;
	/* data for VBO */
	vertexBuffer = new GLfloat[mesh->n_vertices() * 3];
	vertexNormalBuffer = new GLfloat[mesh->n_vertices() * 3];
	/* data for EBO */
	elementBuffer = new GLuint[mesh->n_faces() * 3 * (EDGES - 2)];

	std::map<Veridx, GLuint> vhandles2vindices;
	
	/* export data for VBO */
	GLuint pivot = 0;
	for (PolyArrayMesh::VertexIter iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter, ++pivot)
	{
		Veridx vhandle = *iter;
		vhandles2vindices[vhandle] = pivot;
		memcpy_s(vertexBuffer + pivot * 3, 3 * sizeof(GLfloat), mesh->point(vhandle).data(), 3 * sizeof(GLfloat));
		memcpy_s(vertexNormalBuffer + pivot * 3, 3 * sizeof(GLfloat), mesh->normal(vhandle).data(), 3 * sizeof(GLfloat));
	}
	vertexSize = pivot;
	/* export data for EBO */
	pivot = 0;
	for (PolyArrayMesh::FaceIter iter = mesh->faces_begin(); iter != mesh->faces_end(); ++iter)
	{
		PolyArrayMesh::FaceHandle fhandle = *iter;
		PolyArrayMesh::ConstFaceVertexIter cfviter = mesh->cfv_iter(fhandle);
		/* for a face with n edges, element buffer is
		   (0, 1, 2),  (0, 2, 3),  (0, 3, 4), ..., (0, n-2, n-1)
		   rearrange is 
		   0, 1, (2, 0, 2), (3, 0, 3), ..., (n-2, 0, n-2), n-1
		*/
		Veridx v0 = *cfviter;
		elementBuffer[pivot++] = vhandles2vindices[*(cfviter++)];
		elementBuffer[pivot++] = vhandles2vindices[*(cfviter++)];
		for (size_t i = 2; i <= EDGES - 2; ++i, ++cfviter)
		{
			elementBuffer[pivot++] = vhandles2vindices[*cfviter];
			elementBuffer[pivot++] = vhandles2vindices[v0];
			elementBuffer[pivot++] = vhandles2vindices[*cfviter];
		}
		elementBuffer[pivot++] = vhandles2vindices[*cfviter];
	}
	elementSize = pivot;
	return ;
}

void ClothPiece::exportFaceNorm3fBuffer(GLfloat *& fBarycenterBuffer, GLfloat *& fNormalBuffer, GLuint & faceSize)
const
{
	// TODO
	PolyArrayMesh* mesh = this->PolyMesh;
	fBarycenterBuffer = new GLfloat[mesh->n_faces() * 3];
	fNormalBuffer = new GLfloat[mesh->n_faces() * 3];

	OpenMesh::FPropHandleT<Point3f> fprop_normal = mesh->face_normals_pph();
	GLuint pivot = 0;
	for (auto iter = mesh->faces_begin(); iter != mesh->faces_end(); ++iter, ++pivot)
	{
		PolyArrayMesh::FaceHandle fhd = *iter;
		// face normal
		Point3f normal = mesh->property(fprop_normal, fhd);
		memcpy_s(fNormalBuffer + pivot * 3, 3 * sizeof(GLfloat), normal.data(), 3 * sizeof(GLfloat));
		// face barycenter
		Point3f barycenter(0.0f, 0.0f, 0.0f);
		GLuint edge_cnt = 0;
		for (auto viter = mesh->cfv_begin(fhd); viter != mesh->cfv_end(fhd); ++viter, ++edge_cnt)
		{
			barycenter += mesh->point(*viter);
		}
		barycenter /= (edge_cnt > 1 ? edge_cnt : 1);
		memcpy_s(fBarycenterBuffer + pivot * 3, 3 * sizeof(GLfloat), barycenter.data(), 3 * sizeof(GLfloat));
	}
	faceSize = pivot;
	return;
}

//template <typename PropType> 
//OpenMesh::VPropHandleT<PropType> ClothPiece::addVProp(std::string propName, PropType value)
//{
//	OpenMesh::VPropHandleT<PropType> vprop_handle;
//	// new property handle
//	PolyMesh.add_property(vprop_handle, propName);
//	PolyMesh.property(vprop_handle).set_persistent(true);
//	// initial with val
//	for (PolyArrayMesh::VertexIter iter = PolyMesh.vertices_begin(); iter != PolyMesh.vertices_end(); ++iter)
//		PolyMesh.property(vprop_handle, *iter) = value;
//	//OpenMesh::VPropHandleT<Point3f> vph;
//	//std::cout << PolyMesh.get_property_handle(vph, "planar_coord_3f") << std::endl;
//	return vprop_handle;
//}

template <typename PropType> 
OpenMesh::VPropHandleT<PropType> ClothPiece::addVProp(
	std::string propName, 
	std::function<PropType (Veridx)> vhandle2value
	)
{
	OpenMesh::VPropHandleT<PropType> vprop_handle;
	// new property handle
	PolyMesh->add_property(vprop_handle, propName);
	PolyMesh->property(vprop_handle).set_persistent(true);
	// initial with val
	for (PolyArrayMesh::VertexIter iter = PolyMesh->vertices_begin(); iter != PolyMesh->vertices_end(); ++iter)
		PolyMesh->property(vprop_handle, *iter) = vhandle2value(*iter);
	//OpenMesh::VPropHandleT<Point3f> vph;
	//std::cout << PolyMesh.get_property_handle(vph, "planar_coord_3f") << std::endl;
	return vprop_handle;
}



bool ClothPiece::useVTexCoord2DAsVPlanarCoord3f()
{
	if (!PolyMesh->has_vertex_texcoords2D())
		return false;
	OpenMesh::VPropHandleT<Point3f> vprop_handle = this->addVProp<Point3f>(
		"planar_coord_3f", 
		[&](Veridx vhandle) -> Point3f {
				auto tex2d = PolyMesh->texcoord2D(vhandle);
				return Point3f(tex2d[0], tex2d[1], 0.0f); }
		);
	return true;
}

bool ClothPiece::getVPlanarCoord3f(OpenMesh::VPropHandleT<Point3f> & vph)
{
	return PolyMesh->get_property_handle(vph, "planar_coord_3f");
}

#endif

#ifdef CGAL_BASED

#include "BasicOperations.h"

void ClothPiece::import(const Mesh mesh)
{
	/* load vertexes */
	std::map<GLuint, Veridx> vindices2vhandles;
	for (size_t i = 0; i < mesh.vertices.size(); ++i)
	{
		vindices2vhandles[i] = this->PolyMesh->add_vertex(
			Point3f(mesh.vertices[i].Position[0], mesh.vertices[i].Position[1], mesh.vertices[i].Position[2]));
	}
	/* set faces */
	std::vector<Veridx> face_vhandles;
	for (GLuint vindex : mesh.indices)
	{
		face_vhandles.push_back(vindices2vhandles[vindex]);
		if (face_vhandles.size() % EDGES == 0)
		{
			this->PolyMesh->add_face(face_vhandles);
			face_vhandles.clear();
		}
	}
	/* load texcoord 2d */
	SurfaceMesh3f::Property_map<Veridx, Point3f> texCoords = PolyMesh->add_property_map<Veridx, Point3f>(pname_texCoords).first;
	for (size_t _i = 0; _i < mesh.vertices.size(); ++_i)
	{
		texCoords[vindices2vhandles[_i]] = Point3f(mesh.vertices[_i].TexCoords[0], mesh.vertices[_i].TexCoords[1], 0.0f);
	}
	/* compute face normal */
	/* generate vertex normal conditioning on face normal */
	//PolyArrayMesh & mesh = *PolyMesh;
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals =
		PolyMesh->add_property_map<Faceidx, Vec3f>(pname_faceNormals, CGAL::NULL_VECTOR).first;
	SurfaceMesh3f::Property_map<Veridx, Vec3f> vertexNormals =
		PolyMesh->add_property_map<Veridx, Vec3f>(pname_vertexNormals, CGAL::NULL_VECTOR).first;
	CGAL::Polygon_mesh_processing::compute_normals(*PolyMesh, vertexNormals, faceNormals,
		CGAL::Polygon_mesh_processing::parameters::vertex_point_map(PolyMesh->points()).geom_traits(Kernelf()));

	/* initial sizes */
	VERTEX_SIZE = PolyMesh->number_of_vertices();
	FACE_SIZE = PolyMesh->number_of_faces();
	EDGE_SIZE = PolyMesh->number_of_edges();

	/* initial global indices */
	vertices2indices = new std::map<Veridx, GLuint>();
	faces2indices = new std::map<Faceidx, GLuint>();
	edges2indices = new std::map<Edgeidx, GLuint>();
	GLuint index = 0;
	BOOST_FOREACH(Veridx vhd, PolyMesh->vertices())
	{
		(*vertices2indices)[vhd] = index++;
	}
	index = 0;
	BOOST_FOREACH(Faceidx fhd, PolyMesh->faces())
	{
		(*faces2indices)[fhd] = index++;
	}
	index = 0;
	BOOST_FOREACH(Edgeidx ehd, PolyMesh->edges())
	{
		(*edges2indices)[ehd] = index++;
	}

	std::cout << "INFO::LOAD MESH " << std::endl;
	std::cout << "> #polygon " << EDGES
		<< ", #vertices " << PolyMesh->number_of_vertices()
		<< ", #edges " << PolyMesh->number_of_edges()
		<< ", #halfedges " << PolyMesh->number_of_halfedges()
		<< ", #faces " << PolyMesh->number_of_faces() << std::endl;
}

Eigen::VectorXf ClothPiece::getPositions() const
{
	Eigen::VectorXf positions = Eigen::VectorXf(VERTEX_SIZE * 3);
	for (auto iter = PolyMesh->vertices_begin(); iter != PolyMesh->vertices_end(); ++iter)
	{
		Eigen::Vector3f pos_eigen;
		Veridx vh = *iter;
		copy_v3f(pos_eigen, PolyMesh->point(vh));
		positions.block<3, 1>(vertices2indices->at(vh) * 3, 0) = pos_eigen;
	}
	return positions;
}

void ClothPiece::setPositions(Eigen::VectorXf const & positions)
{
	BOOST_FOREACH(Veridx viter, PolyMesh->vertices())
	{
		Point3f pos_cgal;
		Eigen::Vector3f pos_eigen = positions.block<3, 1>(vertices2indices->at(viter) * 3, 0);
		copy_v3f(pos_cgal, pos_eigen);
		PolyMesh->point(viter) = pos_cgal;
	}
	/* after changing position
	* update normals for consistence
	*/
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals =
		PolyMesh->property_map<Faceidx, Vec3f>(pname_faceNormals).first;
	//CGAL::Polygon_mesh_processing::compute_face_normals(mesh, faceNormals);
	SurfaceMesh3f::Property_map<Veridx, Vec3f> vertexNormals =
		PolyMesh->property_map<Veridx, Vec3f>(pname_vertexNormals).first;
	//CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vertexNormals);
	CGAL::Polygon_mesh_processing::compute_normals(*PolyMesh, vertexNormals, faceNormals,
		CGAL::Polygon_mesh_processing::parameters::vertex_point_map(PolyMesh->points()).geom_traits(Kernelf()));

}

/* export data for VBO and EBO for drawing */
void ClothPiece::exportPos3fNorm3fBuffer(
	GLfloat* & vertexBuffer, GLfloat* & vertexNormalBuffer, GLuint & vertexSize,
	GLuint* & elementBuffer, GLuint & elementSize) const
{
	SurfaceMesh3f* mesh = this->PolyMesh;
	SurfaceMesh3f::Property_map<Veridx, Vec3f> vertexNormals = PolyMesh->property_map<Veridx, Vec3f>(pname_vertexNormals).first;
	/* data for VBO */
	vertexBuffer = new GLfloat[mesh->number_of_vertices() * 3];
	vertexNormalBuffer = new GLfloat[mesh->number_of_vertices() * 3];
	/* data for EBO */
	elementBuffer = new GLuint[mesh->number_of_faces() * 3 * (EDGES - 2)];

	std::map<Veridx, GLuint> vhandles2vindices;

	/* export data for VBO */
	GLuint pivot = 0;
	BOOST_FOREACH(Veridx vhandle, mesh->vertices())
	{
		vhandles2vindices[vhandle] = pivot;
		// TODO to be tested
		memcpy_s(vertexBuffer + pivot * 3, 3 * sizeof(GLfloat), &(mesh->point(vhandle)), 3 * sizeof(GLfloat));
		memcpy_s(vertexNormalBuffer + pivot * 3, 3 * sizeof(GLfloat), &(vertexNormals[vhandle]), 3 * sizeof(GLfloat));
		++pivot;
	}
	vertexSize = pivot;
	/* export data for EBO */
	pivot = 0;
	BOOST_FOREACH(Faceidx fhandle, mesh->faces())
	{
		CGAL::Vertex_around_face_circulator<SurfaceMesh3f> cfviter(mesh->halfedge(fhandle), *mesh);
		/* for a face with n edges, element buffer is
		(0, 1, 2),  (0, 2, 3),  (0, 3, 4), ..., (0, n-2, n-1)
		rearrange is
		0, 1, (2, 0, 2), (3, 0, 3), ..., (n-2, 0, n-2), n-1
		*/
		Veridx v0 = *cfviter;
		elementBuffer[pivot++] = vhandles2vindices[*(cfviter++)];
		elementBuffer[pivot++] = vhandles2vindices[*(cfviter++)];
		for (size_t _i = 2; _i <= EDGES - 2; ++_i, ++cfviter)
		{
			elementBuffer[pivot++] = vhandles2vindices[*cfviter];
			elementBuffer[pivot++] = vhandles2vindices[v0];
			elementBuffer[pivot++] = vhandles2vindices[*cfviter];
		}
		elementBuffer[pivot++] = vhandles2vindices[*cfviter];
	}
	elementSize = pivot;
	return;
}

void ClothPiece::exportFaceNorm3fBuffer(GLfloat *& fBarycenterBuffer, GLfloat *& fNormalBuffer, GLuint & faceSize) const
{
	// TODO
	SurfaceMesh3f* mesh = this->PolyMesh;
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals = mesh->property_map<Faceidx, Vec3f>(pname_faceNormals).first;

	fBarycenterBuffer = new GLfloat[mesh->number_of_faces() * 3];
	fNormalBuffer = new GLfloat[mesh->number_of_faces() * 3];

	GLuint pivot = 0;
	BOOST_FOREACH(Faceidx fhd, mesh->faces())
	{
		// face normal
		Vec3f normal = faceNormals[fhd];
		memcpy_s(fNormalBuffer + pivot * 3, 3 * sizeof(GLfloat), &normal, 3 * sizeof(GLfloat));
		// face barycenter
		Vec3f barycenter(0.0f, 0.0f, 0.0f);
		size_t edge_cnt = 0;
		BOOST_FOREACH(Veridx viter, vertices_around_face(mesh->halfedge(fhd), *mesh))
		{
			Point3f v = mesh->point(viter);
			barycenter = barycenter + Vec3f(v.x(), v.y(), v.z());
			++edge_cnt;
		}
		barycenter = barycenter * (1.0f / (edge_cnt > 1 ? edge_cnt : 1));
		memcpy_s(fBarycenterBuffer + pivot * 3, 3 * sizeof(GLfloat), &barycenter, 3 * sizeof(GLfloat));
		++pivot;
	}
	faceSize = pivot;
	return;
}

bool ClothPiece::useVTexCoord2DAsVPlanarCoord3f()
{
	// TODO add return false case
	SurfaceMesh3f::Property_map<Veridx, Point3f> vprop_handle = 
		PolyMesh->add_property_map<Veridx, Point3f>(pname_vertexPlanarCoords).first;
	SurfaceMesh3f::Property_map<Veridx, Point3f> texCoords =
		PolyMesh->property_map<Veridx, Point3f>(pname_texCoords).first;
	BOOST_FOREACH(Veridx vhd, PolyMesh->vertices())
	{
		vprop_handle[vhd] = texCoords[vhd];
	}
	return true;
}

bool ClothPiece::getVPlanarCoord3f(SurfaceMesh3f::Property_map<Veridx, Point3f> & vph)
{
	// TODO add return false case
	vph = PolyMesh->property_map<Veridx, Point3f>(pname_vertexPlanarCoords).first;
	return true;
}

#endif