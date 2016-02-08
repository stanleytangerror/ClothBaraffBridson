#ifndef CLOTH_MODEL
#define CLOTH_MODEL

#include "Model.h"

#include <assimp\types.h>
#include <map>
#include <functional>

#define _USE_MATH_DEFINES
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

	bool useVTexCoord2DAsVPlanarCoord3f();

	bool ClothPiece::getVPlanarCoord3f(OpenMesh::VPropHandleT<OpenMesh::Vec3f> & vph);

private:
	const GLuint EDGES;
	PolyArrayMesh* PolyMesh = new PolyArrayMesh();

	//template <typename PropType>
	//OpenMesh::VPropHandleT<PropType> ClothPiece::addVProp(std::string propName, PropType value);

	template <typename PropType>
	OpenMesh::VPropHandleT<PropType> ClothPiece::addVProp(
		std::string propName,
		std::function<PropType(PolyArrayMesh::VertexHandle)> vhandle2value
		);


};

#endif


