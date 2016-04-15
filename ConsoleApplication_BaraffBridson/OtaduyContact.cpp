#include "OtaduyContact.h"

#include <map>

void TriangleTree::exportAABBoxPositions(GLfloat *& verticesBuffer, GLuint & pointSize)
{
	pointSize = m_tree->size() * 2;
	verticesBuffer = new GLfloat[pointSize * 3];
	std::list<AABBTree<Triangle3f, Point3f>::NodeType *> const * boxes = m_tree->getBoxes();
	int pivot = 0;
	for (auto iter = boxes->begin(); iter != boxes->end(); ++iter)
	{
		AABBox<Point3f> * box = (*iter)->first;
		Point3f const p = box->minCor();
		verticesBuffer[pivot++] = p.x();
		verticesBuffer[pivot++] = p.y();
		verticesBuffer[pivot++] = p.z();
		Point3f const q = box->maxCor();
		verticesBuffer[pivot++] = q.x();
		verticesBuffer[pivot++] = q.y();
		verticesBuffer[pivot++] = q.z();
	}
}

std::list<std::pair<typename AABBTree<Triangle3f, Point3f>::Index, float> > *
TriangleTree::pointTriangleContactDetection(Point3f const & point, GLfloat tolerance)
{
	return m_tree->contactDetection(point, tolerance);
}

void OtaduyContact::pointTriangleDetection(GLfloat tolerance)
{
	m_pointTriangeContact = new std::map<Veridx, OtaduyContact::Contact2Triangle *>();
	auto mesh = m_clothPiece->getMesh();
	for (Veridx vid : mesh->vertices())
	{
		(*m_pointTriangeContact)[vid] = m_rigidBodyBoxTree->pointTriangleContactDetection(mesh->point(vid), tolerance);
	}
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
