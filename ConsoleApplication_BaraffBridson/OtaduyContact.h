#ifndef OTADUY_CONTACT_H
#define OTADUY_CONTACT_H

#include "SurfaceMeshObject.h"
#include "Scene.h"
#include "AABBTree\AABBTree.h"

class FaceTree
{
public:
	typedef std::list<std::pair<typename AABBTree<Triangle3f, Point3f>::Index, Eigen::Vector3f> *> Contact2Triangle;

	FaceTree(SurfaceMesh3f * mesh, FaceIter2Triangle3fAABBoxPair::VPropertyMap * propertyMap = nullptr) :
		m_mesh(mesh) // TODO , m_treeSceneIndex(-1)
	{
		Faceiter begin = mesh->faces_begin();
		Faceiter end = mesh->faces_end();
		m_tree = new AABBTree<Triangle3f, Point3f>(begin, end,
			FaceIter2Triangle3fAABBoxPair(this->m_mesh, propertyMap));
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

	Contact2Triangle * pointTriangleContactDetection(Point3f const & point, GLfloat tolerance);

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
	typedef std::list<std::pair<typename AABBTree<Segment3f, Point3f>::Index, Eigen::Vector2f> *> Contact2Edge;

	EdgeTree(SurfaceMesh3f * mesh, EdgeIter2Segment3fAABBoxPair::VPropertyMap * propertyMap = nullptr) :
		m_mesh(mesh) // TODO , m_treeSceneIndex(-1)
	{
		Edgeiter begin = mesh->edges_begin();
		Edgeiter end = mesh->edges_end();
		m_tree = new AABBTree<Segment3f, Point3f>(begin, end,
			EdgeIter2Segment3fAABBoxPair(this->m_mesh, propertyMap));
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

	Contact2Edge * edgeEdgeContactDetection(Segment3f const & segment, GLfloat tolerance);

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
	
	FaceTree * m_clothPieceFaceBoxTree;
	FaceTree * m_rigidBodyFaceBoxTree;

	EdgeTree * m_clothPieceEdgeBoxTree;
	EdgeTree * m_rigidBodyEdgeBoxTree;

	explicit OtaduyContact(SurfaceMeshObject * clothPiece, SurfaceMeshObject * rigidBody):
		m_clothPiece(clothPiece), m_rigidBody(rigidBody)
	{
		auto predictPositionMap = clothPiece->getMesh()->property_map<Veridx, Point3f>(clothPiece->pname_vertexPredictPositions).first;
		m_clothPieceFaceBoxTree = new FaceTree(clothPiece->getMesh(), &predictPositionMap);
		m_clothPieceEdgeBoxTree = new EdgeTree(clothPiece->getMesh(), &predictPositionMap);
		auto rigidbodyPositionMap = rigidBody->getMesh()->property_map<Veridx, Point3f>(rigidBody->pname_vertexPredictPositions).first;
		m_rigidBodyFaceBoxTree = new FaceTree(rigidBody->getMesh(), &rigidbodyPositionMap);
		m_rigidBodyEdgeBoxTree = new EdgeTree(rigidBody->getMesh(), &rigidbodyPositionMap);
	}

	void pointTriangleDetection(GLfloat tolerance);
	void edgeEdgeDetection(GLfloat tolerance);

	void applyPointImpulse();
	void applyEdgeImpulse();

	void exportContactPoints(GLfloat * & buffer, GLuint & size);
	void exportContactEdges(GLfloat * & buffer, GLuint & size);

private:
	SurfaceMeshObject * const m_clothPiece;
	SurfaceMeshObject * const m_rigidBody;
	
	std::map<Veridx, FaceTree::Contact2Triangle *> * m_pointTriangeContact;
	std::map<Edgeidx, EdgeTree::Contact2Edge *> * m_edgeEdgeContact;

	void pointImpluse(Veridx vid, Faceidx fid_s, Eigen::Vector3f & coord);
	void edgeImpluse(Edgeidx eid, Edgeidx eid_s, Eigen::Vector2f & coord);
};



#endif
