#ifndef AABBTREE_H
#define AABBTREE_H

#include "../BasicTypes.h"

#include <boost\geometry.hpp>
#include <boost\geometry\index\rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost\foreach.hpp>

#include <algorithm>
#include <list>

inline Point3f & interpolate(Point3f const & a, Point3f const & b, float t)
{
	return Point3f(
		a.x() * t + b.x() * (1.0f - t), 
		a.y() * t + b.y() * (1.0f - t), 
		a.z() * t + b.z() * (1.0f - t));
}

class AABBox
{
public:
	AABBox(Point3f minCor, Point3f maxCor) :
		m_minCor(minCor), m_maxCor(maxCor)
	{
		m_center = interpolate(m_minCor, m_maxCor, 0.5f);
	}

	float distance(AABBox const & rhs) const
	{
		if (intersection(rhs))
			return 0.0f;

		float delta_x = (std::max)((std::min)(rhs.m_maxCor.x() - this->m_minCor.x(), this->m_maxCor.x() - rhs.m_minCor.x()), 0.0f);
		float delta_y = (std::max)((std::min)(rhs.m_maxCor.y() - this->m_minCor.y(), this->m_maxCor.y() - rhs.m_minCor.y()), 0.0f);
		float delta_z = (std::max)((std::min)(rhs.m_maxCor.z() - this->m_minCor.z(), this->m_maxCor.z() - rhs.m_minCor.z()), 0.0f);

		return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
	}

	bool intersection(AABBox const & rhs) const
	{
		if (this->m_minCor.x() > rhs.m_maxCor.x() || this->m_maxCor.x() < rhs.m_minCor.x())
			return false;
		if (this->m_minCor.y() > rhs.m_maxCor.y() || this->m_maxCor.y() < rhs.m_minCor.y())
			return false;
		if (this->m_minCor.z() > rhs.m_maxCor.z() || this->m_maxCor.z() < rhs.m_minCor.z())
			return false;
		return true;
	}
	
	friend AABBox const & operator+(AABBox const & lhs, AABBox const & rhs)
	{
		return AABBox(
			Point3f((std::min)(lhs.m_minCor.x(), rhs.m_minCor.x()),
				(std::min)(lhs.m_minCor.y(), rhs.m_minCor.y()),
				(std::min)(lhs.m_minCor.z(), rhs.m_minCor.z())),
			Point3f((std::min)(lhs.m_maxCor.x(), rhs.m_maxCor.x()),
				(std::min)(lhs.m_maxCor.y(), rhs.m_maxCor.y()),
				(std::min)(lhs.m_maxCor.z(), rhs.m_maxCor.z()))
			);
	}

	Point3f const & minCor()
	{
		return m_minCor;
	}

	Point3f const & maxCor()
	{
		return m_maxCor;
	}

private:
	Point3f const m_minCor;
	Point3f const m_maxCor;
	Point3f m_center;

};

//template <typename Object, typename Primitive>
//bool intersection(Object const & obj, Primitive const & pri);
//
//template <>
//bool intersection<Point3f, Triangle3f>(Point3f const & point, Triangle3f const & triangle)
//{
//	
//	return false;
//}

template <typename Primitive>
class AABBTree
{
public:
	typedef std::pair<AABBox *, Primitive *> NodeType;

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

private:
	std::list<NodeType *> * tree;

};

struct FaceIter2Triangle3fAABBoxPair
{
	SurfaceMesh3f const * const m_mesh;
	FaceIter2Triangle3fAABBoxPair(SurfaceMesh3f * mesh) : m_mesh(mesh) {}

	AABBTree<Triangle3f>::NodeType * operator() (/*SurfaceMesh3f const * mesh, */Faceiter const & iter) const
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
		auto box = new AABBox(Point3f(minx, miny, minz), Point3f(maxx, maxy, maxz));
		auto tri = new Triangle3f(ps[0], ps[1], ps[2]);
		//std::cout << "box" << std::endl << box->minCor() << std::endl << box->maxCor() << std::endl;
		//std::cout << "triangle" << std::endl << ps[0] << std::endl << ps[1] << std::endl << ps[2] << std::endl;
		return new AABBTree<Triangle3f>::NodeType(box, tri);
	}

};


////BOOST_GEOMETRY_REGISTER_POINT_3D(Point3f, float, CGAL::Simple_cartesian<float>, x, y, z)
////BOOST_GEOMETRY_REGISTER_BOX(CGAL::Bbox_3, Point3f, bottom, top)
//
//namespace bg = boost::geometry;
//namespace bgi = boost::geometry::index;
//namespace bgm = boost::geometry::model;
//
//
///* see http://stackoverflow.com/questions/14195111/r-trees-should-i-reinvent-the-wheel
//* and http://www.boost.org/doc/libs/1_60_0/libs/geometry/doc/html/geometry/reference/adapted/register/boost_geometry_register_box.html
//*/
//
//
//// OR use predefined ones
////typedef bgm::point<float, 3, bg::cs::cartesian> Point;
//typedef bgm::box<Point3f> AABB;
//
//class TriangleBox
//{
//public:
//	TriangleBox(Triangle3f & triangle) : 
//		m_triangle(triangle)
//	{
//		
//		//m_aabb.min_corner();
//		//m_aabb.max_corner();
//	}
//	
//	virtual ~TriangleBox() {}
//	
//	AABB const & bounding_box() const
//	{
//		return m_aabb; 
//	}
//
//private:
//	Triangle3f & m_triangle;
//	AABB m_aabb;
//};
//
//// Tell the rtree how to extract the AABB from the Shape
//namespace boost {
//	namespace geometry {
//		namespace index {
//
//			template <>
//			struct indexable< boost::shared_ptr<TriangleBox> >
//			{
//				typedef boost::shared_ptr<TriangleBox> V;
//				typedef AABB const& result_type;
//
//				result_type operator()(V const& v) const 
//				{
//					return v->bounding_box(); 
//				}
//			};
//
//		}
//	}
//} // namespace boost::geometry::index
//
//  void test()
//  {
//  	bgi::rtree< boost::shared_ptr<TriangleBox>, bgi::rstar<32> > rtree;
//  }

//
//struct AABBox
//{
//	Point3f top;
//	Point3f bottom;
//
//};
//
//struct Node
//{
//	Node *parent;
//	Node *children[2];
//
//	// these will be explained later
//	bool childrenCrossed;
//	AABB aabb;
//	AABB *data;
//	Node(void)
//		: parent(nullptr)
//		, data(nullptr)
//	{
//		children[0] = nullptr;
//		children[1] = nullptr;
//	}
//
//	bool IsLeaf(void) const
//	{
//		return children[0] = nullptr;
//	}
//
//	// make this ndoe a branch
//	void SetBranch(Node *n0, Node *n1)
//	{
//		n0->parent = this;
//		n1->parent = this;
//
//		children[0] = n0;
//		children[1] = n1;
//	}
//
//	// make this node a leaf
//	void SetLeaf(AABB *data)
//	{
//		// create two-way link
//		this->data = data;
//		data->userData = this;
//
//		children[0] = nullptr;
//		children[1] = nullptr;
//	}
//
//	void UpdateAABB(float margin)
//	{
//		if (IsLeaf())
//		{
//			// make fat AABB
//			const Vec3 marginVec(margin, margin, margin);
//			aabb.minPoint = data->minPoint - marginVec;
//			aabb.maxPoint = data->maxPoint + marginVec;
//		}
//		else
//			// make union of child AABBs of child nodes
//			aabb =
//			children[0]->aabb.Union(children[1]->aabb);
//	}
//
//	Node *GetSibling(void) const
//	{
//		return
//			this == parent->children[0]
//			? parent->children[1]
//			: parent->children[0];
//	}
//};
//
//class AABBTree : public Broadphase
//{
//public:
//
//	AABBTree(void)
//		: m_root(nullptr)
//		, m_margin(0.2f) // 20cm
//	{ }
//
//	virtual void Add(AABB *aabb);
//	virtual void Remove(AABB *aabb);
//	virtual void Update(void);
//	virtual ColliderPairList &ComputePairs(void);
//	virtual Collider *Pick(const Vec3 &point) const;
//	virtual Query(const AABB &aabb, ColliderList &out) const;
//	virtual RayCastResult RayCast(const Ray3 &ray) const;
//
//private:
//
//	typedef std::vector<Node *> NodeList;
//
//	void UpdateNodeHelper(Node *node, NodeList &invalidNodes);
//	void InsertNode(Node *node, Node **parent);
//	void RemoveNode(Node *node);
//	void ComputePairsHelper(Node *n0, Node *n1);
//	void ClearChildrenCrossFlagHelper(Node *node);
//	void CrossChildren(Node *node);
//
//	Node *m_root;
//	ColliderPairList m_pairs;
//	float m_margin;
//	NodeList m_invalidNodes;
//};

#endif
