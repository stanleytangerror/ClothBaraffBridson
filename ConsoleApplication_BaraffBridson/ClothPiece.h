#ifndef CLOTH_MODEL
#define CLOTH_MODEL

#include "Model.h"

#include <assimp\types.h>
#include <map>
#include <functional>

//#define OPENMESH_BASED
#define CGAL_BASED

#ifdef OPENMESH_BASED

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriArrayMesh;
typedef OpenMesh::PolyMesh_ArrayKernelT<> PolyArrayMesh;

class ClothPiece
{
public:
	ClothPiece(GLuint edges) :
		EDGES(edges) {}

	PolyArrayMesh* getMesh()
	{
		return PolyMesh;
	}

	void import(const Mesh mesh);
	
	void exportPos3fNorm3fBuffer(
		GLfloat* & vertexBuffer, GLfloat* & vertexNormalBuffer, GLuint & vertexSize, 
		GLuint* & elementBuffer, GLuint & elementSize);

	void exportFaceNorm3fBuffer(
		GLfloat* & fBarycenterBuffer, GLfloat* & fNormalBuffer, GLuint & faceSize);

	bool useVTexCoord2DAsVPlanarCoord3f();

	bool getVPlanarCoord3f(OpenMesh::VPropHandleT<OpenMesh::Vec3f> & vph);

private:
	const GLuint EDGES;
	PolyArrayMesh* PolyMesh = new PolyArrayMesh();

	//template <typename PropType>
	//OpenMesh::VPropHandleT<PropType> ClothPiece::addVProp(std::string propName, PropType value);

	template <typename PropType>
	OpenMesh::VPropHandleT<PropType> addVProp(
		std::string propName,
		std::function<PropType(PolyArrayMesh::VertexHandle)> vhandle2value
		);

};
#endif

#ifdef CGAL_BASED

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL\Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <boost/foreach.hpp>

// kernel type
typedef CGAL::Simple_cartesian<float> Kernelf;
// primitive types
typedef Kernelf::FT FT;
typedef Kernelf::Point_3 Point3f;
typedef Kernelf::Vector_3 Vec3f;
typedef Kernelf::Triangle_3 Triangle3f;
typedef Kernelf::Segment_3 Segment3f;
// surface mesh type
typedef CGAL::Surface_mesh<Point3f> SurfaceMesh3f;
// primitive index types
typedef SurfaceMesh3f::Vertex_index Veridx;
typedef SurfaceMesh3f::Face_index Faceidx;
typedef SurfaceMesh3f::Edge_index Edgeidx;
typedef SurfaceMesh3f::Halfedge_index Halfedgeidx;

class ClothPiece
{
public:
	ClothPiece(GLuint edges) :
		EDGES(edges), PolyMesh(new SurfaceMesh3f())
	{}

	SurfaceMesh3f* getMesh()
	{
		return PolyMesh;
	}

	void import(const Mesh mesh);

	void exportPos3fNorm3fBuffer(
		GLfloat* & vertexBuffer, GLfloat* & vertexNormalBuffer, GLuint & vertexSize,
		GLuint* & elementBuffer, GLuint & elementSize) const;

	void exportFaceNorm3fBuffer(
		GLfloat* & fBarycenterBuffer, GLfloat* & fNormalBuffer, GLuint & faceSize) const;

	bool useVTexCoord2DAsVPlanarCoord3f();

	bool getVPlanarCoord3f(SurfaceMesh3f::Property_map<Veridx, Point3f> & vph);

	const std::string pname_texCoords = "v:texture_coordinates";
	const std::string pname_vertexPlanarCoords = "v:vertex_planar_coordinates";
	const std::string pname_vertexNormals = "v:vertex_normals";
	const std::string pname_faceNormals = "v:face_normals";

private:
	const GLuint EDGES;
	SurfaceMesh3f* PolyMesh;
	/* mesh properties */

};
#endif

#endif


