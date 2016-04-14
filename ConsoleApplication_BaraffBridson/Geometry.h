#ifndef DISTANCE_H
#define DISTANCE_H

#include "BasicTypes.h"

#include <CGAL/squared_distance_3.h>
#include <CGAL\global_functions_spherical_kernel_3.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_3.h>

#ifdef CGAL_USE_GMP
#include <CGAL/Gmpzf.h>
typedef CGAL::Gmpzf ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

typedef CGAL::Polytope_distance_d_traits_3<Kernelf, ET, double> Traits;
typedef CGAL::Polytope_distance_d<Traits>                 Polytope_distance;


float const NEAR_SQUARE_DISTANCE = 1e-30f;

template <typename Primitive, typename RefPrimitive>
float squared_distance(Primitive const & p, RefPrimitive const & rp);

template <>
inline float squared_distance<Point3f, Triangle3f>(Point3f const & point, Triangle3f const & triangle)
{
	Point3f po[1] = { point };
	Point3f tri[3] = { triangle.vertex(0), triangle.vertex(1), triangle.vertex(2) };
	Polytope_distance pd(po, po + 1, tri, tri + 8);
	assert(pd.is_valid());
	double sqdis = CGAL::to_double(pd.squared_distance_numerator()) /
		CGAL::to_double(pd.squared_distance_denominator());
	return float(sqdis);
}

template <>
inline float squared_distance<Segment3f, Segment3f>(Segment3f const & segment, Segment3f const & refSegment)
{
	return CGAL::squared_distance(segment, refSegment);
}

template <>
inline float squared_distance<Point3f, Plane3f>(Point3f const & point, Plane3f const & plane)
{
	return CGAL::squared_distance(point, plane);
}

template <>
inline float squared_distance<Point3f, Point3f>(Point3f const & point, Point3f const & refPoint)
{
	return CGAL::squared_distance(point, refPoint);
}

//template <typename Primitive, typename RefPrimitive>
//bool intersection(Primitive const & p, RefPrimitive const & rp, float tolerance);
//
//template <>
//inline bool intersection<Point3f, Triangle3f>(Point3f const & point, Triangle3f const & triangle, float tolerance)
//{
//	// TODO check if triangle degenerato to
//	// point
//
//	// segment
//
//	// else
//	float sqdis = 0.0f;
//	auto pairPtr = (*iter);
//	AABBox<Point3f> * box = pairPtr->first;
//	Triangle3f const * tri = pairPtr->second;
//	// should near the bounding box
//	if (box->squared_distance(point) >= tolerance)
//		continue;
//	Plane3f plane = tri->supporting_plane();
//	Point3f foot = plane.projection(point);
//	// should near the support plane of the triangle
//	sqdis = squared_distance(point, foot);
//	if (sqdis >= tolerance)
//		continue;
//	float projDis = tolerance - sqdis;
//	/* foot should near the triangle
//	* by solving v0p = t1 * v0v1 + t2 * v0v2
//	* with t1, t2 and (t1+t2) all in [0, 1]
//	* existance of the root:
//	* 1. foot-triangle coplane;
//	* 2. v0v1, v0v2 not coline;
//	*/
//
//	Vec3f v0p = Vec3f(tri->vertex(0), foot);
//	Vec3f v0v1 = Vec3f(tri->vertex(0), tri->vertex(1));
//	Vec3f v0v2 = Vec3f(tri->vertex(0), tri->vertex(2));
//
//	if (vec0 * vec1 >= 0.0f && vec1 * vec2 >= 0.0f && vec2 * vec0 >)
//		sqdis = squared_distance(foot, Segment3f(tri->vertex(0), tri->vertex(1)));
//	if (sqdis >= projDis)
//		continue;
//	sqdis = squared_distance(point, *tri);
//	if (sqdis >= tolerance)
//		continue;
//}

#endif