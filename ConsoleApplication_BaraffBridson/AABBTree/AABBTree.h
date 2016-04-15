#ifndef AABBTREE_H
#define AABBTREE_H

#include "../BasicTypes.h"
#include "../BasicOperations.h"

#include <boost\geometry.hpp>
#include <boost\geometry\index\rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost\foreach.hpp>

#include <algorithm>
#include <list>

/* --------------- AABBox class definition --------------- */

template <typename PointType>
class AABBox
{
public:
	AABBox(PointType minCor, PointType maxCor) :
		m_minCor(minCor), m_maxCor(maxCor)
	{
		m_center = interpolate(m_minCor, m_maxCor, 0.5f);
	}

	float squared_distance(AABBox const & rhs) const
	{
		if (intersection(rhs))
			return 0.0f;

		float delta_x = (std::max)((std::min)(rhs.m_maxCor.x() - this->m_minCor.x(), this->m_maxCor.x() - rhs.m_minCor.x()), 0.0f);
		float delta_y = (std::max)((std::min)(rhs.m_maxCor.y() - this->m_minCor.y(), this->m_maxCor.y() - rhs.m_minCor.y()), 0.0f);
		float delta_z = (std::max)((std::min)(rhs.m_maxCor.z() - this->m_minCor.z(), this->m_maxCor.z() - rhs.m_minCor.z()), 0.0f);

		return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
	}

	template <typename Obj>
	float squared_distance(Obj const & obj)
	{
		BOOST_STATIC_ASSERT(sizeof(T) == 0);
	}

	template <typename Obj>
	bool intersection(Obj const & obj, float tolerance)
	{
		BOOST_STATIC_ASSERT(sizeof(T) == 0);
	}

	bool intersection(AABBox<PointType> const & rhs) const
	{
		if (this->m_minCor.x() > rhs.m_maxCor.x() || this->m_maxCor.x() < rhs.m_minCor.x())
			return false;
		if (this->m_minCor.y() > rhs.m_maxCor.y() || this->m_maxCor.y() < rhs.m_minCor.y())
			return false;
		if (this->m_minCor.z() > rhs.m_maxCor.z() || this->m_maxCor.z() < rhs.m_minCor.z())
			return false;
		return true;
	}
	
	friend AABBox<PointType> const & operator+(AABBox<PointType> const & lhs, AABBox<PointType> const & rhs)
	{
		return AABBox<PointType>(
			PointType((std::min)(lhs.m_minCor.x(), rhs.m_minCor.x()),
				(std::min)(lhs.m_minCor.y(), rhs.m_minCor.y()),
				(std::min)(lhs.m_minCor.z(), rhs.m_minCor.z())),
			PointType((std::min)(lhs.m_maxCor.x(), rhs.m_maxCor.x()),
				(std::min)(lhs.m_maxCor.y(), rhs.m_maxCor.y()),
				(std::min)(lhs.m_maxCor.z(), rhs.m_maxCor.z()))
			);
	}

	PointType const & minCor()
	{
		return m_minCor;
	}

	PointType const & maxCor()
	{
		return m_maxCor;
	}


private:
	PointType const m_minCor;
	PointType const m_maxCor;
	PointType m_center;

};

template <> template <>
bool AABBox<Point3f>::intersection<Point3f>(Point3f const & point, float tolerance);

template <> template <>
float AABBox<Point3f>::squared_distance<Point3f>(Point3f const & point);


/* --------------- AABBTree class definition --------------- */

template <typename Primitive, typename PointType>
class AABBTree
{
public:
	typedef std::pair<AABBox<PointType> *, Primitive *> NodeType;
	typedef int Index;

	template <typename IterType, typename toPair>
	AABBTree(IterType & begin, IterType const & end, toPair & topair):
		tree(new std::list<NodeType *>())
	{
		for (; begin != end; ++begin)
		{
			NodeType * p = topair(begin);
			tree->push_back(p);
		}
	}

	std::list<NodeType *> const * getBoxes()
	{
		return tree;
	}

	~AABBTree()
	{
		for (NodeType * item : tree)
		{
			delete item->first;
			delete item->second;
			delete item;
		}
	}

	int size()
	{
		return tree->size();
	}

	template <typename Obj>
	std::list<std::pair<Index, float> > * contactDetection(Obj const & obj, float tolerance);

private:
	std::list<NodeType *> * tree;

};

/* --------------- conversions implementations --------------- */

struct FaceIter2Triangle3fAABBoxPair
{
	SurfaceMesh3f const * const m_mesh;
	FaceIter2Triangle3fAABBoxPair(SurfaceMesh3f * mesh) : m_mesh(mesh) {}

	AABBTree<Triangle3f, Point3f>::NodeType * operator() (/*SurfaceMesh3f const * mesh, */Faceiter const & iter) const
	{ 
		Faceidx fid = *iter;
		//std::cout << "face index " << fid << std::endl;
		std::vector<Point3f> ps;
		ps.reserve(3);
		float maxx = NEGATIVE_MAX_FLOAT;
		float maxy = NEGATIVE_MAX_FLOAT;
		float maxz = NEGATIVE_MAX_FLOAT;
		float minx = POSITIVE_MAX_FLOAT;
		float miny = POSITIVE_MAX_FLOAT;
		float minz = POSITIVE_MAX_FLOAT;
		//std::cout << "initial" << std::endl << minx << " " << miny << " " << minz << std::endl
		//	<< maxx << " " << maxy << " " << maxz << std::endl;
		auto range = m_mesh->vertices_around_face(m_mesh->halfedge(fid));
		//CGAL::Vertex_around_face_iterator<SurfaceMesh3f> vbegin , vend;
		for (auto vbegin = range.begin(); vbegin != range.end(); ++vbegin)
		{
			Veridx vid = *vbegin;
			//std::cout << "vertex index" << vid << std::endl;
			Point3f p = m_mesh->point(vid);
			ps.push_back(p);
			//std::cout << "point" << std::endl << p << std::endl;
			minx = (std::min)(minx, p.x());
			miny = (std::min)(miny, p.y());
			minz = (std::min)(minz, p.z());
			maxx = (std::max)(maxx, p.x());
			maxy = (std::max)(maxy, p.y());
			maxz = (std::max)(maxz, p.z());
			//std::cout << "procedure" << std::endl << minx << " " << miny << " " << minz << std::endl
			//	<< maxx << " " << maxy << " " << maxz << std::endl;
		}
		assert(ps.size() == 3);
		auto box = new AABBox<Point3f>(Point3f(minx, miny, minz), Point3f(maxx, maxy, maxz));
		auto tri = new Triangle3f(ps[0], ps[1], ps[2]);
		//std::cout << "box" << std::endl << box->minCor() << std::endl << box->maxCor() << std::endl;
		//std::cout << "triangle" << std::endl << ps[0] << std::endl << ps[1] << std::endl << ps[2] << std::endl;
		return new AABBTree<Triangle3f, Point3f>::NodeType(box, tri);
	}

};

#endif