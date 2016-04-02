#include "OtaduyContact.h"

void TriangleTree::exportAABBoxPositions(GLfloat *& verticesBuffer, GLuint & pointSize)
{
	pointSize = m_tree->size() * 2;
	verticesBuffer = new GLfloat[pointSize * 3];
	std::list<AABBTree<Triangle3f>::NodeType *> const * boxes = m_tree->getBoxes();
	int pivot = 0;
	for (auto iter = boxes->begin(); iter != boxes->end(); ++iter)
	{
		AABBox * box = (*iter)->first;
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
