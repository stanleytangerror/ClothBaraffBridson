#include "ClothPiece.h"
#include <iostream>

void ClothPiece::import(const Mesh mesh)
{
	/* load vertexes */
	std::map<GLuint, PolyArrayMesh::VertexHandle> vindices2vhandles;
	for (size_t i = 0; i < mesh.vertices.size(); ++i)
	{
		vindices2vhandles[i] = this->PolyMesh->add_vertex(
			PolyArrayMesh::Point(mesh.vertices[i].Position[0], mesh.vertices[i].Position[1], mesh.vertices[i].Position[2]));
	}
	/* set faces */
	std::vector<PolyArrayMesh::VertexHandle> face_vhandles;
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
{
	PolyArrayMesh* mesh = this->PolyMesh;
	/* data for VBO */
	vertexBuffer = new GLfloat[mesh->n_vertices() * 3];
	vertexNormalBuffer = new GLfloat[mesh->n_vertices() * 3];
	/* data for EBO */
	elementBuffer = new GLuint[mesh->n_faces() * 3 * (EDGES - 2)];

	std::map<PolyArrayMesh::VertexHandle, GLuint> vhandles2vindices;
	
	/* export data for VBO */
	GLuint pivot = 0;
	for (PolyArrayMesh::VertexIter iter = mesh->vertices_begin(); iter != mesh->vertices_end(); ++iter, ++pivot)
	{
		PolyArrayMesh::VertexHandle vhandle = *iter;
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
		PolyArrayMesh::VertexHandle v0 = *cfviter;
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
//	//OpenMesh::VPropHandleT<OpenMesh::Vec3f> vph;
//	//std::cout << PolyMesh.get_property_handle(vph, "planar_coord_3f") << std::endl;
//	return vprop_handle;
//}

template <typename PropType> 
OpenMesh::VPropHandleT<PropType> ClothPiece::addVProp(
	std::string propName, 
	std::function<PropType (PolyArrayMesh::VertexHandle)> vhandle2value
	)
{
	OpenMesh::VPropHandleT<PropType> vprop_handle;
	// new property handle
	PolyMesh->add_property(vprop_handle, propName);
	PolyMesh->property(vprop_handle).set_persistent(true);
	// initial with val
	for (PolyArrayMesh::VertexIter iter = PolyMesh->vertices_begin(); iter != PolyMesh->vertices_end(); ++iter)
		PolyMesh->property(vprop_handle, *iter) = vhandle2value(*iter);
	//OpenMesh::VPropHandleT<OpenMesh::Vec3f> vph;
	//std::cout << PolyMesh.get_property_handle(vph, "planar_coord_3f") << std::endl;
	return vprop_handle;
}



bool ClothPiece::useVTexCoord2DAsVPlanarCoord3f()
{
	if (!PolyMesh->has_vertex_texcoords2D())
		return false;
	OpenMesh::VPropHandleT<OpenMesh::Vec3f> vprop_handle = this->addVProp<OpenMesh::Vec3f>(
		"planar_coord_3f", 
		[&](PolyArrayMesh::VertexHandle vhandle) -> OpenMesh::Vec3f {
				auto tex2d = PolyMesh->texcoord2D(vhandle);
				return OpenMesh::Vec3f(tex2d[0], tex2d[1], 0.0f); }
		);
	return true;
}

bool ClothPiece::getVPlanarCoord3f(OpenMesh::VPropHandleT<OpenMesh::Vec3f> & vph)
{
	return PolyMesh->get_property_handle(vph, "planar_coord_3f");
}

