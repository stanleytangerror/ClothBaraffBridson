#ifndef OTADUY_CONTACT_H
#define OTADUY_CONTACT_H

#include "ClothPiece.h"
#include "Scene.h"
#include "AABBTree\AABBTree.h"

class TriangleTree
{
public:
	TriangleTree(SurfaceMesh3f * mesh) :
		m_mesh(mesh) // TODO , m_treeSceneIndex(-1)
	{
		Faceiter begin = mesh->faces_begin();
		Faceiter end = mesh->faces_end();
		m_tree = new AABBTree<Triangle3f, Point3f>(begin, end,
			FaceIter2Triangle3fAABBoxPair(this->m_mesh));
		//	[this](Faceiter const & iter)->AABBTree<Triangle3f>::NodeType *
		//{
		//	auto p = FaceIter2Triangle3fAABBoxPair()(this->m_mesh, iter);
		//	std::cout << "in lambda " << p->first->minCor().x() << std::endl;
		//	return p;
		//});

	}

	AABBTree<Triangle3f, Point3f> const * getTree()
	{
		return m_tree;
	}

	void exportAABBoxPositions(GLfloat * & verticesBuffer, GLuint & pointSize);

	//void pointContactDetection(GLfloat tolerance);

	std::list<std::pair<typename AABBTree<Triangle3f, Point3f>::Index, float> > *
		pointTriangleContactDetection(Point3f const & point, GLfloat tolerance);

private:
	//static auto toPri = [](Faceiter iter) -> Triangle3f const & {return Triangle3f(face); };
	//static auto toBox = [](Faceiter iter) -> AABBox const & {};
	SurfaceMesh3f * const m_mesh;
	AABBTree<Triangle3f, Point3f> * m_tree;
	// TODO Scene::Index m_treeSceneIndex;

};

class OtaduyContact
{
public:
	typedef std::list<std::pair<typename AABBTree<Triangle3f, Point3f>::Index, float> > Contact2Triangle;
	
	OtaduyContact(ClothPiece * clothPiece):
		m_clothPiece(clothPiece), m_triangleTree(new TriangleTree(clothPiece->getMesh()))
	{}

	std::map<Veridx, Contact2Triangle *> * point2triangleDetection(GLfloat tolerance);

private:
	ClothPiece * const m_clothPiece;
	TriangleTree * m_triangleTree;
	
};



#endif
