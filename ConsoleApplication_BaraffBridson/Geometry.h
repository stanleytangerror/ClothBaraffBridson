#ifndef DISTANCE_H
#define DISTANCE_H

#include "BasicTypes.h"

#include <CGAL/squared_distance_3.h>
#include <Eigen/Dense>

#ifdef USE_HULL_POINT_TRIANGLE_DISTANCE
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
#endif

/* ---------------- squared distance -------------------- */

template <typename Primitive, typename RefPrimitive>
float squared_distance(Primitive const & p, RefPrimitive const & rp) 
{
	return POSITIVE_MAX_FLOAT;
}

/* WARNING: just for approximation
 * return distance between point and triangle's support plane,
 * precise method may go to http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
 */
template <>
inline float squared_distance<Point3f, Triangle3f>(Point3f const & point, Triangle3f const & triangle)
{
#ifdef USE_HULL_POINT_TRIANGLE_DISTANCE
	Point3f po[1] = { point };
	Point3f tri[3] = { triangle.vertex(0), triangle.vertex(1), triangle.vertex(2) };
	Polytope_distance pd(po, po + 1, tri, tri + 3);
	assert(pd.is_valid());
	double sqdis = CGAL::to_double(pd.squared_distance_numerator()) /
		CGAL::to_double(pd.squared_distance_denominator());
	return float(sqdis);
#else
	Plane3f plane = triangle.supporting_plane();
	return squared_distance(point, plane);
#endif
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

/* ---------------- intersection -------------------- */
template <typename Primitive, typename RefPrimitive>
bool intersection(Primitive const & p, RefPrimitive const & rp, float tolerance) 
{
	return false;
}

template <>
bool intersection<Point3f, Triangle3f>(Point3f const & point, Triangle3f const & triangle, float tolerance)
{
	Point3f const & v1 = triangle.vertex(0);
	Point3f const & v2 = triangle.vertex(1);
	Point3f const & v3 = triangle.vertex(2);
	bool d12 = squared_distance(v1, v2) < DISTANCE_OVERLAP_SQUARED_THRESHOLD;
	bool d23 = squared_distance(v2, v3) < DISTANCE_OVERLAP_SQUARED_THRESHOLD;
	bool d31 = squared_distance(v3, v1) < DISTANCE_OVERLAP_SQUARED_THRESHOLD;
	// point
	if (d12 && d23 && d31)
	{
		return (squared_distance(point, v1) < tolerance) ? true : false;
	}
	// segment
	else if (d12 || d23 || d31)
	{
		if (d12)
			return (squared_distance(point, new Segment3f(v2, v3)) < tolerance) ? true : false;
		else if (d23)
			return (squared_distance(point, new Segment3f(v3, v1)) < tolerance) ? true : false;
		else
			return (squared_distance(point, new Segment3f(v1, v2)) < tolerance) ? true : false;

	}
	// triangle
	else
	{
		Plane3f plane = triangle.supporting_plane();
		float sqdis = squared_distance(point, plane);
		if (sqdis > tolerance)
			return false;

		Point3f foot = plane.projection(point);

		Eigen::Vector3f x13 = displacement(v1, v3);
		Eigen::Vector3f x23 = displacement(v2, v3);
		Eigen::Vector3f xf3 = displacement(foot, v3);
		float x13tx13 = x13.squaredNorm();
		float x13tx23 = x13.transpose() * x23;
		float x23tx23 = x23.squaredNorm();

		Eigen::Matrix3f A;
		A << x13tx13, x13tx23, 0.0f,
			x13tx23, x23tx23, 0.0f,
			1.0f, 1.0f, 1.0f;
		Eigen::Vector3f b;
		b << x13.transpose() * xf3, x23.transpose() * xf3, 1.0f;
		Eigen::Vector3f localCoord = A.householderQr().solve(b);
		if (localCoord[0] > -tolerance && localCoord[0] < 1.0f + tolerance &&
			localCoord[1] > -tolerance && localCoord[1] < 1.0f + tolerance &&
			localCoord[2] > -tolerance && localCoord[2] < 1.0f + tolerance)
		{
			return true;
		}
		return false;
	}
}

template <>
bool intersection<Segment3f, Segment3f>(Segment3f const & seg1, Segment3f const & seg2, float tolerance)
{
	Point3f v1 = seg1.start();
	Point3f v2 = seg1.end();
	Point3f v3 = seg2.start();
	Point3f v4 = seg2.end();
	Eigen::Vector3f x21 = displacement(v2, v1);
	Eigen::Vector3f x31 = displacement(v3, v1);
	Eigen::Vector3f x43 = displacement(v4, v3);

	// parallel
	if (x21.cross(x43).squaredNorm() < DISTANCE_OVERLAP_SQUARED_THRESHOLD)
	{
		return (intersection(AABBoxOf<Point3f, Segment3f>(seg1), 
			AABBoxOf<Point3f, Segment3f>(seg2), tolerance)) 
			? true : false;
	}
	else
	{
		float x21tx21 = x21.squaredNorm();
		float x21tx43 = x21.transpose() * x43;
		float x43tx43 = x43.squaredNorm();

		Eigen::Matrix2f A;
		A << x21tx21, -x21tx43,
			-x21tx43, x43tx43;
		Eigen::Vector2f b;
		b << x21.transpose() * x31, -x43.transpose() * x31;
		Eigen::Vector2f x = A.householderQr().solve(b);
		if (x[0] < 0.0f || x[0] > 1.0f || x[1] < 0.0f || x[1] > 1.0f)
			return false;
		return true;
	}
}

#endif