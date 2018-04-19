#include "Profile.h"
#include <chrono>
#include <fstream>
#include <string>

namespace
{
	long long GetNanoSecondCount()
	{
		return std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::high_resolution_clock::now().time_since_epoch()
			).count();
	}

	static const std::string profileFile = "E:/Microsoft Visual Studio 2015/Workspace/ConsoleApplication_BaraffBridson/log/LogFile.txt";

	void Log(const std::string & line)
	{
		std::ofstream outfile;
		outfile.open(profileFile, std::ios_base::app);
		outfile << line;
	}
}

//#define DEBUG_PROFILER

ScopedProfiler::ScopedProfiler(const Functor & onProfiled)
	: mStart(GetNanoSecondCount())
	, mOnProfiled(onProfiled)
{
#ifdef DEBUG_PROFILER
	Log("start " + std::to_string(mStart) + " ");
#endif
}

ScopedProfiler::~ScopedProfiler()
{
	TimeCount end = GetNanoSecondCount();

#ifdef DEBUG_PROFILER
	Log("end " + std::to_string(end) + " ");
#endif

	mOnProfiled(mStart, end);
}

void Temp::ProfileNewFrame()
{
	Log("\n");
}

void Temp::ProfileResult(const ScopedProfiler::TimeCount start, const ScopedProfiler::TimeCount end)
{
	Log(std::to_string(double(end - start) / 1000000.0) + ", ");
}

