#ifndef OTADUY_CONTACT_H
#define OTADUY_CONTACT_H

#include "SurfaceMeshObject.h"
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

	//void exportAABBoxPositions(GLfloat * & verticesBuffer, GLuint & pointSize);

	//void pointContactDetection(GLfloat tolerance);

	std::list<typename AABBTree<Triangle3f, Point3f>::Index> *
		pointTriangleContactDetection(Point3f const & point, GLfloat tolerance);

private:
	//static auto toPri = [](Faceiter iter) -> Triangle3f const & {return Triangle3f(face); };
	//static auto toBox = [](Faceiter iter) -> AABBox const & {};
	SurfaceMesh3f * const m_mesh;
	AABBTree<Triangle3f, Point3f> * m_tree;
	// TODO Scene::Index m_treeSceneIndex;

};


class EdgeTree
{
public:
	EdgeTree(SurfaceMesh3f * mesh) :
		m_mesh(mesh) // TODO , m_treeSceneIndex(-1)
	{
		Edgeiter begin = mesh->edges_begin();
		Edgeiter end = mesh->edges_end();
		m_tree = new AABBTree<Segment3f, Point3f>(begin, end,
			EdgeIter2Segment3fAABBoxPair(this->m_mesh));
		//	[this](Faceiter const & iter)->AABBTree<Triangle3f>::NodeType *
		//{
		//	auto p = FaceIter2Triangle3fAABBoxPair()(this->m_mesh, iter);
		//	std::cout << "in lambda " << p->first->minCor().x() << std::endl;
		//	return p;
		//});

	}

	AABBTree<Segment3f, Point3f> const * getTree()
	{
		return m_tree;
	}

	//void exportAABBoxPositions(GLfloat * & verticesBuffer, GLuint & pointSize);

	//void pointContactDetection(GLfloat tolerance);

	std::list<typename AABBTree<Segment3f, Point3f>::Index> *
		edgeEdgeContactDetection(Segment3f const & segment, GLfloat tolerance);

private:
	//static auto toPri = [](Faceiter iter) -> Triangle3f const & {return Triangle3f(face); };
	//static auto toBox = [](Faceiter iter) -> AABBox const & {};
	SurfaceMesh3f * const m_mesh;
	AABBTree<Segment3f, Point3f> * m_tree;
	// TODO Scene::Index m_treeSceneIndex;

};


class OtaduyContact
{
public:
	typedef std::list<typename AABBTree<Triangle3f, Point3f>::Index> Contact2Triangle;
	
	TriangleTree * m_clothPieceFaceBoxTree;
	TriangleTree * m_rigidBodyFaceBoxTree;

	EdgeTree * m_clothPieceEdgeBoxTree;
	EdgeTree * m_rigidBodyEdgeBoxTree;

	explicit OtaduyContact(SurfaceMeshObject * clothPiece, SurfaceMeshObject * rigidBody):
		m_clothPiece(clothPiece), m_rigidBody(rigidBody),
		m_clothPieceFaceBoxTree(new TriangleTree(clothPiece->getMesh())),
		m_rigidBodyFaceBoxTree(new TriangleTree(rigidBody->getMesh()))
	{}

	void pointTriangleDetection(GLfloat tolerance);

	void generatePointTriangleConstraints();

	void exportContactPoints(GLfloat * & buffer, GLuint & size);

private:
	SurfaceMeshObject * const m_clothPiece;
	SurfaceMeshObject * const m_rigidBody;
	
	std::map<Veridx, Contact2Triangle *> * m_pointTriangeContact;
	
};



#endif
