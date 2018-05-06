#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "BaraffDynamics.h"
#include "Scene.h"
#include "EventManager.h"
#include "Camera.h"
#include "Clock.h"

class Simulator
{
public:

	Simulator()
		: viewer(new FOVControl())
	{} 

	// WARNING: should be called explicitly
	void init();

	void run();

private:

	SurfaceMeshObject * clothPiece;
	std::vector<SurfaceMeshObject *> mRigidBodies;
	BaraffDynamics * clothDynamics;
	OtaduyContact * contactHandler;
	EventManager * eventManager;
	FOVControl * viewer;

	Scene::Index contactSceneIndex;

	void simulateInternal();

	void pauseEventHandle(bool const * const keyMask);

};

#endif