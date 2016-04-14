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
	return m_tree->contaceDetection(point, tolerance);
}

std::map<Veridx, OtaduyContact::Contact2Triangle *> *
OtaduyContact::point2triangleDetection(GLfloat tolerance)
{
	std::map<Veridx, OtaduyContact::Contact2Triangle *> * contacts_pointTriangle =
		new std::map<Veridx, OtaduyContact::Contact2Triangle *>();
	auto mesh = m_clothPiece->getMesh();
	for (Veridx vid : mesh->vertices())
	{
		(*contacts_pointTriangle)[vid] = m_triangleTree->pointTriangleContactDetection(mesh->point(vid), tolerance);
	}
	return contacts_pointTriangle;
}
