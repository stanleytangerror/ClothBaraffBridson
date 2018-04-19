#ifndef CLOCK_H
#define CLOCK_H

#include "OpenGLContext.h"

#include <ctime>
#include <functional>
#include <vector>

class FrameCounter;

class Clock
{
public:
	Clock(bool isPaused = false)
		: isPaused(isPaused)
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

	void Tick(float deltaTime);

	void PushFrameCounter(FrameCounter * const frameCounter);

private:
	std::clock_t * stdClock;
	bool isPaused;
	std::vector<FrameCounter *>	mCounters;

public:
	static Clock * Instance()
	{
		if (!Clock::msClock)
			Clock::msClock = new Clock(true);

		return Clock::msClock;
	}

private:
	static Clock* msClock;

};

class FrameCounter
{
public:
	using			Functor = std::function<void()>;

	FrameCounter(int frameCount, const Functor & onHit);
	virtual			~FrameCounter() {}

	virtual	void	Tick();

	bool			IsAlive() const { return mAlive; }

protected:
	Functor	mOnHit;
	int		mFrameCount;
	bool	mAlive;
};

#endif

