#pragma once

#include <functional>
#include "BasicTypes.h"

class SurfaceMeshObject;

class ContactHandler
{
public:
	using Resolver = std::function<void(SurfaceMeshObject*, const Veridx&, const Faceidx&, const Eigen::Vector3f& faceNormal, const Eigen::Vector3f&)>;

	ContactHandler(SurfaceMeshObject * clothPiece, const std::vector<SurfaceMeshObject *>& colliders);
	virtual ~ContactHandler() {}

	void SetResolver(const Resolver& resolver) { mResolver = resolver; }
	void Resolve();

protected:
	SurfaceMeshObject *					mClothPiece;
	std::vector<SurfaceMeshObject *>	mRigidBodies;
	Resolver							mResolver;
};
