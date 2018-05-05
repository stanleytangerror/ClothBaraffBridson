#pragma once

#include <functional>
#include <vector>
#include <Eigen\Core>
#include <numeric>
#include <iostream>

#pragma optimize("", off)
template<typename T> // e.g. T = triangle index
class SpatialHashing 
{
public:
	struct AABB
	{
		AABB()
			//: mMin((std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)())
			//, mMax(-(std::numeric_limits<float>::max)(), -(std::numeric_limits<float>::max)(), -(std::numeric_limits<float>::max)())
			: mMin(1000.f * Eigen::Vector3f::Ones())
			, mMax(-1000.f * Eigen::Vector3f::Ones())
		{}

		AABB(const AABB& other) 
			: mMin(other.mMin)
			, mMax(other.mMax)
		{}

		Eigen::Vector3f mMin;
		Eigen::Vector3f mMax;
	};

	using ToAABB = std::function<AABB(const T &)>;
	
	struct Cell
	{
		std::vector<T> mPrimitives;
		void Add(const T& primitive) 
		{
			//if (std::find(mPrimitives.begin(), mPrimitives.end(), primitive) == mPrimitives.end())
				mPrimitives.push_back(primitive); 
		}
	};

	SpatialHashing(const std::vector<T> & primitives, const ToAABB & toBound, float gridLen, unsigned int hashBase)
		: mGridLen(gridLen)
		, mInvGridLength(1.0f / gridLen)
		, mHashBase(hashBase)
		, mGrids(hashBase * hashBase * hashBase)
	{
		for (const T& primitive : primitives)
		{
			const AABB & aabb = toBound(primitive);

			for (float x = aabb.mMin.x() - mGridLen; x <= aabb.mMax.x() + mGridLen; x += mGridLen)
				for (float y = aabb.mMin.y() - mGridLen; y <= aabb.mMax.y() + mGridLen; y += mGridLen)
					for (float z = aabb.mMin.z() - mGridLen; z <= aabb.mMax.z() + mGridLen; z += mGridLen)
						mGrids[HashToGrid(Eigen::Vector3f(x, y, z))].Add(primitive);
		}
	}

	const int HashToGrid(const Eigen::Vector3f & position) const
	{
		const Eigen::Vector3f temp = position * mInvGridLength;
		int x = ((int)std::abs(temp.x())) % mHashBase;
		int y = ((int)std::abs(temp.y())) % mHashBase;
		int z = ((int)std::abs(temp.z())) % mHashBase;
		return (x * mHashBase + y) * mHashBase + z;
	}

	const Cell & Query(const Eigen::Vector3f & position) const
	{
		return mGrids[HashToGrid(position)];
	}

private:
	float				mGridLen;
	float				mInvGridLength;
	unsigned int		mHashBase;
	std::vector<Cell>	mGrids;
};
