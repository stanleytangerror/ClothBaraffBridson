#include "AABBTree.h"
#include "../Geometry.h"

/* --------- AABBox specialization implementations ------------ */

template <> template <>
bool AABBox<Point3f>::intersection<Point3f>(Point3f const & point, float tolerance)
{
	if (point.x() + tolerance < m_minCor.x() || point.x() - tolerance > m_maxCor.x())
		return false;
	if (point.y() + tolerance < m_minCor.y() || point.y() - tolerance > m_maxCor.y())
		return false;
	if (point.z() + tolerance < m_minCor.z() || point.z() - tolerance > m_maxCor.z())
		return false;
	return true;
}

template <> template <>
float AABBox<Point3f>::squared_distance<Point3f>(Point3f const & point)
{
	if (this->intersection<Point3f>(point, 0.0f))
		return 0.0f;

	float delta_x = (std::max)((std::max)(this->m_minCor.x() - point.x(), point.x() - this->m_maxCor.x()), 0.0f);
	float delta_y = (std::max)((std::max)(this->m_minCor.y() - point.y(), point.y() - this->m_maxCor.y()), 0.0f);
	float delta_z = (std::max)((std::max)(this->m_minCor.z() - point.z(), point.z() - this->m_maxCor.z()), 0.0f);

	//std::cout << "delta x " << delta_x << " delta y " << delta_y << " delta z " << delta_z << std::endl;

	return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
}

template <>
AABBox<Point3f> const & AABBoxOf<Point3f, Segment3f>(Segment3f const & segment)
{
	AABBox<Point3f> box(segment.start(), segment.start());
	return box + segment.end();
}


/* --------- AABBTree specialization implementations ------------ */

template<> template<>
std::list<typename AABBTree<Triangle3f, Point3f>::Index> *
AABBTree<Triangle3f, Point3f>::contactDetection<Point3f>(Point3f const & point, float tolerance)
{
	typedef AABBTree<Triangle3f, Point3f>::Index Idx;
	std::list<Idx> * result = new std::list<Idx>();
	Idx idx = 0;
	for (auto iter = tree->begin(); iter != tree->end(); ++iter, ++idx)
	{
		auto pairPtr = (*iter);
		AABBox<Point3f> * box = pairPtr->first;
		Triangle3f const * tri = pairPtr->second;
		float sqdis = 0.0f;
		// should near the bounding box
		if (box->squared_distance(point) >= tolerance)
			continue;
		//std::cout << "box " << std::endl
		//	<< box->minCor() << std::endl << box->maxCor() << std::endl;
		if (!intersection(point, *tri, tolerance))
			continue;
		result->push_back(idx);
	}
	return result;
}

template<> template<>
std::list<typename AABBTree<Segment3f, Point3f>::Index> *
AABBTree<Segment3f, Point3f>::contactDetection<Segment3f>(Segment3f const & segment, float tolerance)
{
	typedef AABBTree<Segment3f, Point3f>::Index Idx;
	std::list<Idx> * result = new std::list<Idx>();
	Idx idx = 0;
	for (auto iter = tree->begin(); iter != tree->end(); ++iter, ++idx)
	{
		auto pairPtr = (*iter);
		AABBox<Point3f> * box = pairPtr->first;
		Segment3f const * seg = pairPtr->second;
		float sqdis = 0.0f;
		// should near the bounding box
		if (!box->intersection(AABBoxOf<Point3f, Segment3f>(segment)))
			continue;
		//std::cout << "box " << std::endl
		//	<< box->minCor() << std::endl << box->maxCor() << std::endl;

		//sqdis = squared_distance(*seg, segment);
		//if (sqdis > tolerance)
		//	continue;
		if (!intersection(*seg, segment, tolerance))
			continue;
		result->push_back(idx);
	}
	return result;
}
