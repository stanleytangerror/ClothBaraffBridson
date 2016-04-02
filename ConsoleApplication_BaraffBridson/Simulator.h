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

	explicit Simulator():
		viewer(new FOVControl()), clock(new Clock())
	{} 

	// WARNING: should be called explicitly
	void init();

	void run();

private:

	ClothPiece * clothPiece;
	BaraffDynamics * clothDynamics;
	EventManager * eventManager;
	FOVControl * viewer;
	Clock * clock;

	GLuint loopCount;

	Scene::Index boxSceneIndex;

	void updateData();

	void pauseEventHandle(bool const * const keyMask);

};

#endif