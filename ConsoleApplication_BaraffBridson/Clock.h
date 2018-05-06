#ifndef CLOCK_H
#define CLOCK_H

#include "OpenGLContext.h"

#include <ctime>
#include <functional>
#include <vector>

#include "Singleton.h"

class FrameCounter;

class Clock : public Singleton<Clock>
{
public:
	Clock()
		: isPaused(true)
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

