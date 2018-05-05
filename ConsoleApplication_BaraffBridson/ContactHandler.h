#pragma once

class SurfaceMeshObject;

class ContactHandler
{
public:
	ContactHandler(SurfaceMeshObject * clothPiece, SurfaceMeshObject * rigidBody);
	virtual ~ContactHandler() {}

	void Resolve();

protected:
	SurfaceMeshObject * mClothPiece;
	SurfaceMeshObject * mRigidBody;
};
