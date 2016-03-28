#ifndef CLOCK_H
#define CLOCK_H

#include "OpenGLContext.h"

#include <ctime>

class Clock
{
public:
	Clock() :
		isPaused(false)
	{ }

	void pause()
	{
		isPaused = true;
	}

	void resume()
	{
		isPaused = false;
	}

	bool paused()
	{
		return isPaused;
	}

private:
	std::clock_t * stdClock;
	bool isPaused;

};

#endif

