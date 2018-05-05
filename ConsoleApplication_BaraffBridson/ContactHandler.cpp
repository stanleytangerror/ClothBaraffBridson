#include "ContactHandler.h"
#include <array>
#include "BasicOperations.h"
#include "SurfaceMeshObject.h"
#include "SpatialHashing.h"
#include "DebugRenderer.h"
#include "OpenGLContext.h"

namespace
{
	const std::array<GLfloat, 108> cubeVertices = {
		// Positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f,  1.0f
	};

	class DebugContact : public IRenderBufferProvider
	{
	public:
		DebugContact() {}

		void Push(const Eigen::Vector3f & point)
		{
			mBuffer.push_back((GLfloat)point.x());
			mBuffer.push_back((GLfloat)point.y());
			mBuffer.push_back((GLfloat)point.z());
		}

		void Push(const std::array<Eigen::Vector3f, 3> & triangle)
		{
			for (int i = 0; i < 3; ++i)
				Push(triangle[i]);
		}

		void PushSinglePoint(const Eigen::Vector3f & point, const float size)
		{
			static std::array<Eigen::Vector3f, 3> triangle;
			triangle[0] = point + Eigen::Vector3f(size, 0.f, 0.f);
			triangle[1] = point + Eigen::Vector3f(0.0f, size, 0.f);
			triangle[2] = point + Eigen::Vector3f(0.0f, 0.f, size);
			Push(triangle);
		}

		void Push(const SpatialHashing<Faceidx>::AABB & aabb)
		{
			Eigen::Vector3f center = 0.5f * (aabb.mMax + aabb.mMin);
			Eigen::Vector3f extend = 0.5f * (aabb.mMax - aabb.mMin);
			
			for (int i = 0; i < cubeVertices.size(); i += 3)
			{
				Push(Eigen::Vector3f(
					cubeVertices[i + 0] * extend.x(), 
					cubeVertices[i + 1] * extend.y(), 
					cubeVertices[i + 2] * extend.z()) 
					+ center);
			}
		}

		void Clear()
		{
			mBuffer.clear();
		}

		virtual const GLfloat * Buffer() const override
		{
			return mBuffer.data();
		}

		virtual const GLuint BufferSize() const override
		{
			return mBuffer.size();
		}

		std::vector<GLfloat>	mBuffer;
	};

	static DebugContact * gDebugContact = new DebugContact;

	SpatialHashing<Faceidx>::AABB FaceAABB(const SurfaceMesh3f * mesh, const Faceidx & faceIdx)
	{
		SpatialHashing<Faceidx>::AABB aabb;

		static int count = 0;
		bool debug = false; // (count++) % 37 == 0;

		std::array<Eigen::Vector3f, 3> triangle;
		int i = 0;
		CGAL::Vertex_around_face_circulator<SurfaceMesh3f> cfviter(mesh->halfedge(faceIdx), *mesh), done(cfviter);
		do
		{
			const auto & pos = mesh->point(*cfviter);
			aabb.mMin.x() = std::min<float>(aabb.mMin.x(), pos.x());
			aabb.mMin.y() = std::min<float>(aabb.mMin.y(), pos.y());
			aabb.mMin.z() = std::min<float>(aabb.mMin.z(), pos.z());
			aabb.mMax.x() = std::max<float>(aabb.mMax.x(), pos.x());
			aabb.mMax.y() = std::max<float>(aabb.mMax.y(), pos.y());
			aabb.mMax.z() = std::max<float>(aabb.mMax.z(), pos.z());

			copy_v3f(triangle[i], pos);
			//triangle[i].x() = std::abs<float>(triangle[i].x());
			//triangle[i].y() = std::abs<float>(triangle[i].y());
			//triangle[i].z() = std::abs<float>(triangle[i].z());

			++cfviter;
			++i;
		} while (cfviter != done);

		if (debug)
		{
			gDebugContact->Push(triangle);
			//gDebugContact->Push(aabb);

			float mGridLen = 0.3f;
			int mHashBase = 67;

			for (float x = aabb.mMin.x() - mGridLen; x <= aabb.mMax.x() + mGridLen; x += mGridLen)
				for (float y = aabb.mMin.y() - mGridLen; y <= aabb.mMax.y() + mGridLen; y += mGridLen)
					for (float z = aabb.mMin.z() - mGridLen; z <= aabb.mMax.z() + mGridLen; z += mGridLen)
					{
						const Eigen::Vector3f temp = Eigen::Vector3f(x, y, z) * (1.f / mGridLen);
						int i = ((int)std::abs(temp.x())) % mHashBase;
						int j = ((int)std::abs(temp.y())) % mHashBase;
						int k = ((int)std::abs(temp.z())) % mHashBase;

						SpatialHashing<Faceidx>::AABB aabb;
						aabb.mMin = Eigen::Vector3f(i * mGridLen, j * mGridLen, k * mGridLen);
						aabb.mMax = Eigen::Vector3f(i * mGridLen + mGridLen, j * mGridLen + mGridLen, k * mGridLen + mGridLen);
						gDebugContact->Push(aabb);
					}

		}

		return aabb;
	}

	bool solve_UniOutsideTrianglePointDistanceConstraint(
		const Eigen::Vector3f& p,
		const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
		const Eigen::Vector3f& n,
		const float& dist,
		Eigen::Vector3f& corr)
	{
		// find barycentric coordinates of closest point on triangle

		float b0 = 1.0f / 3.0f;        // for singular case
		float b1 = b0;
		float b2 = b0;

		Eigen::Vector3f d1 = p1 - p0;
		Eigen::Vector3f d2 = p2 - p0;
		Eigen::Vector3f pp0 = p - p0;
		float a = d1.transpose() * d1;
		float b = d2.transpose() * d1;
		float c = pp0.transpose() * d1;
		float d = b;
		float e = d2.transpose() * d2;
		float f = pp0.transpose() * d2;
		float det = a * e - b * d;

		if (det != 0.0f)
		{
			float s = (c * e - b * f) / det;
			float t = (a * f - c * d) / det;
			b0 = 1.0f - s - t;       // inside triangle
			b1 = s;
			b2 = t;
			if (b0 <= 0.0f || b1 <= 0.0f || b2 <= 0.0f)
			{
				return true;
			}
		}
		Eigen::Vector3f q = p0 * b0 + p1 * b1 + p2 * b2;
		Eigen::Vector3f offset = p - q;
		float offsetMag = offset.norm();

		if (offsetMag < dist)
		{
			//if (Eigen::Vector3f.Dot(n, offset) > 0.0f)
			corr = (q + n * dist - p);
			corr = -corr;
			return true;
		}
		
		return false;
	}

	bool PointTriangleContactResolve(Eigen::Vector3f& position, const SurfaceMesh3f * mesh, const SurfaceMesh3f::Property_map<Faceidx, Vec3f> & faceNormalMap, const Faceidx & faceIdx)
	{
		std::array<Eigen::Vector3f, 3> triangle;
		Eigen::Vector3f normal;
		Eigen::Vector3f corr;
		copy_v3f(normal, faceNormalMap[faceIdx]);

		int i = 0;
		CGAL::Vertex_around_face_circulator<SurfaceMesh3f> cfviter(mesh->halfedge(faceIdx), *mesh), done(cfviter);
		do
		{
			copy_v3f(triangle[i], mesh->point(*cfviter));
			++cfviter;
			++i;
		} while (cfviter != done);

		if (solve_UniOutsideTrianglePointDistanceConstraint(position, triangle[0], triangle[1], triangle[2], normal, 0.05f, corr))
		{
			gDebugContact->Push(triangle);
			position += corr;
			return true;
		}
		return false;
	}
}

ContactHandler::ContactHandler(SurfaceMeshObject * clothPiece, SurfaceMeshObject * rigidBody)
	: mClothPiece(clothPiece)
	, mRigidBody(rigidBody)
{
}

void ContactHandler::Resolve()
{
	if (!mClothPiece || !mRigidBody) return;

	static bool doOnce = (DebugRenderer::Instance()->Push(gDebugContact), true);

	gDebugContact->Clear();

	SurfaceMesh3f * rigidBodyMesh = mRigidBody->getMesh();
	SurfaceMesh3f * clothPieceMesh = mClothPiece->getMesh();

	std::vector<Faceidx> faces;
	BOOST_FOREACH(Faceidx fidx, rigidBodyMesh->faces())
	{
		faces.push_back(fidx);
	}

	SpatialHashing<Faceidx> space(
		faces,
		[rigidBodyMesh](const Faceidx& faceIdx) { return FaceAABB(rigidBodyMesh, faceIdx); },
		0.3f,
		67);

#if 0
	for (int i = 0; i < 10; ++i)
		for (int j = 0; j < 10; ++j)
			for (int k = 0; k < 10; ++k)
			{
				SpatialHashing<Faceidx>::AABB aabb;
				aabb.mMin = Eigen::Vector3f(i * 0.3f, j * 0.3f, k * 0.3f);
				aabb.mMax = Eigen::Vector3f(i * 0.3f + 0.3f, j * 0.3f + 0.3f, k * 0.3f + 0.3f);
				gDebugContact->Push(aabb);
			}
#endif

	SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals = rigidBodyMesh->property_map<Faceidx, Vec3f>(mRigidBody->pname_faceNormals).first;

	BOOST_FOREACH(Veridx vidx, clothPieceMesh->vertices())
	{
		Eigen::Vector3f pos;
		copy_v3f(pos, clothPieceMesh->point(vidx));

		for (const Faceidx & fidx : space.Query(pos).mPrimitives)
		{
			PointTriangleContactResolve(pos, rigidBodyMesh, faceNormals, fidx);
		}
		//copy_v3f(clothPieceMesh->point(vidx), pos);
	}	
}
