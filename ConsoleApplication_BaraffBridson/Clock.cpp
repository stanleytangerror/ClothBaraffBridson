#include "Clock.h"

void Clock::Tick(float deltaTime)
{
	for (auto & counter : mCounters)
	{
		counter->Tick();
		if (!counter->IsAlive())
		{
			delete counter;
			counter = nullptr;
		}
	}

	std::vector<FrameCounter *> newCounters;
	for (auto & counter : mCounters)
	{
		if (counter)
			newCounters.push_back(counter);
	}

	std::swap(newCounters, mCounters);
}

void Clock::PushFrameCounter(FrameCounter * const frameCounter)
{
	if (frameCounter)
		mCounters.push_back(frameCounter);
}

Clock* Clock::msClock = nullptr;

FrameCounter::FrameCounter(int frameCount, const Functor& onHit)
	: mFrameCount(frameCount)
	, mOnHit(onHit)
	, mAlive(true)
{

}

void FrameCounter::Tick()
{
	if (!mAlive) return;

	if (mFrameCount > 0)
	{
		mFrameCount--;
	}
	else
	{
		mAlive = false;
		mOnHit();
	}
}
