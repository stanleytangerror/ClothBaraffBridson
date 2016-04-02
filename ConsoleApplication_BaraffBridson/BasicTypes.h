#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H

#define CGAL_BASED

#ifdef OPENMESH_BASED

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriArrayMesh;
typedef OpenMesh::PolyMesh_ArrayKernelT<> PolyArrayMesh;

#endif

#ifdef CGAL_BASED

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL\Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <boost/foreach.hpp>

#include <Eigen\Core>

#include <map>

// kernel type
typedef CGAL::Simple_cartesian<float> Kernelf;
// primitive types
typedef Kernelf::FT FT;
typedef Kernelf::Point_3 Point3f;
typedef Kernelf::Vector_3 Vec3f;
typedef Kernelf::Triangle_3 Triangle3f;
typedef Kernelf::Segment_3 Segment3f;
// surface mesh type
typedef CGAL::Surface_mesh<Point3f> SurfaceMesh3f;
// primitive index types
typedef SurfaceMesh3f::Vertex_index Veridx;
typedef SurfaceMesh3f::Face_index Faceidx;
typedef SurfaceMesh3f::Edge_index Edgeidx;
typedef SurfaceMesh3f::Halfedge_index Halfedgeidx;
// primitive iterator types
typedef SurfaceMesh3f::Vertex_iterator Veriter;
typedef SurfaceMesh3f::Edge_iterator Edgeiter;
typedef SurfaceMesh3f::Face_iterator Faceiter;

#endif

float const POSITIVE_MAX_FLOAT = (std::numeric_limits<float>::max)();
float const NEGATIVE_MAX_FLOAT = -POSITIVE_MAX_FLOAT;

#endif