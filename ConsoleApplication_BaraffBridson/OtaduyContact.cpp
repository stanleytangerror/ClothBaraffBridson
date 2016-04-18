#include "OtaduyContact.h"
#include "AABBTree\AABBTree.h"

#include <map>

/* -------------- TriangleTree ------------- */

//template<> template<>
//std::list<AABBTree<Triangle3f, Point3f>::Index> *
//AABBTree<Triangle3f, Point3f>::contactDetection<Point3f>(Point3f const & point, float tolerance);

std::list<typename AABBTree<Triangle3f, Point3f>::Index> *
TriangleTree::pointTriangleContactDetection(Point3f const & point, GLfloat tolerance)
{
	return m_tree->contactDetection(point, tolerance);
}

/* -------------- EdgeTree ------------- */

std::list<typename AABBTree<Segment3f, Point3f>::Index> *
EdgeTree::edgeEdgeContactDetection(Segment3f const & segment, GLfloat tolerance)
{
	return m_tree->contactDetection(segment, tolerance);
	//return nullptr;
}

/* -------------- contact ------------- */

void OtaduyContact::pointTriangleDetection(GLfloat tolerance)
{
	m_pointTriangeContact = new std::map<Veridx, OtaduyContact::Contact2Triangle *>();
	auto mesh = m_clothPiece->getMesh();
	for (Veridx vid : mesh->vertices())
	{
		(*m_pointTriangeContact)[vid] = m_rigidBodyFaceBoxTree->pointTriangleContactDetection(mesh->point(vid), tolerance);
	}
}

void OtaduyContact::edgeEdgeDetection(GLfloat tolerance)
{
	m_edgeEdgeContact = new std::map<Edgeidx, OtaduyContact::Contact2Edge *>();
	auto mesh = m_clothPiece->getMesh();
	for (Edgeidx eid : mesh->edges())
	{
		Segment3f seg(mesh->point(mesh->vertex(eid, 0)), mesh->point(mesh->vertex(eid, 1)));
		(*m_edgeEdgeContact)[eid] = m_rigidBodyEdgeBoxTree->edgeEdgeContactDetection(seg, tolerance);
	}
}


void OtaduyContact::generatePointTriangleConstraints()
{

}

void OtaduyContact::exportContactPoints(GLfloat *& buffer, GLuint & size)
{
	size = 0;
	buffer = new GLfloat[m_clothPiece->getVertexSize() * 3];
	int pivot = 0;
	for (auto contact : *m_pointTriangeContact)
	{
		auto colls = contact.second;
		if (colls->empty())
			continue;
		size += 1;
		auto p = m_clothPiece->getMesh()->point(contact.first);
		buffer[pivot++] = p.x();
		buffer[pivot++] = p.y();
		buffer[pivot++] = p.z();
	}
}

void OtaduyContact::exportContactEdges(GLfloat *& buffer, GLuint & size)
{
	size = 0;
	buffer = new GLfloat[m_clothPiece->getEdgeSize() * 2 * 3];
	int pivot = 0;
	auto mesh = m_clothPiece->getMesh();
	for (auto contact : *m_edgeEdgeContact)
	{
		auto colls = contact.second;
		if (colls->empty())
			continue;
		size += 2;
		auto p = mesh->point(mesh->vertex(contact.first, 0));
		buffer[pivot++] = p.x();
		buffer[pivot++] = p.y();
		buffer[pivot++] = p.z();
		p = mesh->point(mesh->vertex(contact.first, 1));
		buffer[pivot++] = p.x();
		buffer[pivot++] = p.y();
		buffer[pivot++] = p.z();
	}
}
