#pragma once

#include <functional>

class ScopedProfiler
{
public:
	using TimeCount = long long;
	using Functor = std::function<void(const TimeCount, const TimeCount)>;

				ScopedProfiler(const Functor & onProfiled);
	virtual		~ScopedProfiler();

protected:
	TimeCount	mStart;
	Functor		mOnProfiled;
};


namespace Temp
{
	void ProfileNewFrame();

	void ProfileResult(const ScopedProfiler::TimeCount start, const ScopedProfiler::TimeCount end);

}

#define PROFILE_SCOPE							\
	ScopedProfiler profiler(Temp::ProfileResult);		\
	(profiler);
