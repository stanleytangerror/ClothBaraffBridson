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

	float delta_x = (std::max)((std::min)(point.x() - this->m_minCor.x(), this->m_maxCor.x() - point.x()), 0.0f);
	float delta_y = (std::max)((std::min)(point.y() - this->m_minCor.y(), this->m_maxCor.y() - point.y()), 0.0f);
	float delta_z = (std::max)((std::min)(point.z() - this->m_minCor.z(), this->m_maxCor.z() - point.z()), 0.0f);

	return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
}

/* --------- AABBTree specialization implementations ------------ */

template<> template<>
std::list<std::pair<typename AABBTree<Triangle3f, Point3f>::Index, float> > *
AABBTree<Triangle3f, Point3f>::contaceDetection(Point3f const & point, float tolerance)
{
	typedef std::pair<typename AABBTree<Triangle3f, Point3f>::Index, float> DisItem;
	std::list<DisItem> * result = new std::list<DisItem>();
	Index idx = 0;
	for (auto iter = tree->begin(); iter != tree->end(); ++iter, ++idx)
	{
		auto pairPtr = (*iter);
		AABBox<Point3f> * box = pairPtr->first;
		Triangle3f const * tri = pairPtr->second;
		float sqdis = 0.0f;
		// should near the bounding box
		if (box->squared_distance(point) >= tolerance)
			continue;
		sqdis = squared_distance(point, *tri);
		if (sqdis >= tolerance)
			continue;
		//Point3f v0 = tri->vertex(0);
		//Point3f v1 = tri->vertex(1);
		//Point3f v2 = tri->vertex(2);
		//Vec3f v0v1 = Vec3f(tri->vertex(0), tri->vertex(1));
		//Vec3f v1v2 = Vec3f(tri->vertex(1), tri->vertex(2));
		//Vec3f v2v0 = Vec3f(tri->vertex(2), tri->vertex(0));
		//float v0v1_sl = v0v1.squared_length();
		//float v1v2_sl = v1v2.squared_length();
		//float v2v0_sl = v2v0.squared_length();
		//if (v0v1_sl < NEAR_SQUARE_DISTANCE
		//	&& v1v2_sl < NEAR_SQUARE_DISTANCE)
		//{
		//	/* triangle degenerate to a point*/
		//	sqdis = squared_distance(point, v0);
		//	if (sqdis >= tolerance)
		//		continue;
		//}
		//else if (v0v1_sl < NEAR_SQUARE_DISTANCE || squared_distance(v1, Segment3f(v2, v0)))
		//{
		//	sqdis = squared_distance(point, Segment3f(v2, v0));
		//	if (sqdis >= tolerance)
		//		continue;
		//}
		//else if (v1v2_sl < NEAR_SQUARE_DISTANCE || squared_distance(v2, Segment3f(v0, v1)))
		//{
		//	sqdis = squared_distance(point, Segment3f(v0, v1));
		//	if (sqdis >= tolerance)
		//		continue;
		//}
		//else if (v2v0_sl < NEAR_SQUARE_DISTANCE || squared_distance(v0, Segment3f(v1, v2)))
		//{
		//	sqdis = squared_distance(point, Segment3f(v1, v2));
		//	if (sqdis >= tolerance)
		//		continue;
		//}
		//else
		//{
		//	/* check distance between point and triangle in 3d */
		//	Plane3f plane = tri->supporting_plane();
		//	Point3f foot = plane.projection(point);
		//	// should near the support plane of the triangle
		//	Vec3f v0p = Vec3f(tri->vertex(0), foot);
		//	Vec3f v1p = Vec3f(tri->vertex(1), foot);
		//	Vec3f v2p = Vec3f(tri->vertex(2), foot);
		//	Vec3f v0v1_n = Vec3f(v0v1.direction());
		//	auto v1v2_n = v1v2.direction();
		//	auto v2v0_n = v2v0.direction();
		//	Vec3f d0 = -v0v1_n + v2v0_n;
		//	sqdis = squared_distance(point, foot);
		//	if (sqdis >= tolerance)
		//		continue;
		//	float projDis = tolerance - sqdis;
		//	/* foot should near the triangle
		//	 * by solving v0p = t1 * v0v1 + t2 * v0v2
		//	 * with t1, t2 and (t1+t2) all in [0, 1]
		//	 * existance of the root:
		//	 * foot point and triangle coplane;
		//	 */


		//	if (vec0 * vec1 >= 0.0f && vec1 * vec2 >= 0.0f && vec2 * vec0 > )
		//		sqdis = squared_distance(foot, Segment3f(tri->vertex(0), tri->vertex(1)));
		//	if (sqdis >= projDis)
		//		continue;
		//	sqdis = squared_distance(point, *tri);
		//	if (sqdis >= tolerance)
		//		continue;
		//}
		result->push_back(DisItem(idx, sqdis));
	}
	return result;
}
