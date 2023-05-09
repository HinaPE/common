#ifndef HINAPE_INTERSECTION_QUERY_ENGINE_H
#define HINAPE_INTERSECTION_QUERY_ENGINE_H

#include "math/math_ext.h"
#include "math/ray.h"
#include "geom/bbox.h"

namespace HinaPE::Util
{

template<typename T>
struct ClosestIntersectionQueryResult3
{
	const T *item = nullptr;
	real distance = Constant::I_REAL_MAX;
};

template<typename T> using ClosestIntersectionDistanceFunc3 = std::function<real(const T &, const mVector3 &)>;
template<typename T> using BoxIntersectionTestFunc3 = std::function<bool(const T &, const mBBox3 &)>;
template<typename T> using RayIntersectionTestFunc3 = std::function<bool(const T &, const mRay3 &)>;
template<typename T> using GetRayIntersectionFunc3 = std::function<real(const T &, const mRay3 &)>;
template<typename T> using IntersectionVisitorFunc3 = std::function<void(const T &)>;

template<typename T>
class IntersectionQueryEngine3
{
public:
	virtual auto intersects(const mBBox3 &box, const BoxIntersectionTestFunc3<T> &testFunc) const -> bool = 0;
	virtual auto intersects(const mRay3 &ray, const RayIntersectionTestFunc3<T> &testFunc) const -> bool = 0;
	virtual void forEachIntersectingItem(const mBBox3 &box, const BoxIntersectionTestFunc3<T> &testFunc, const IntersectionVisitorFunc3<T> &visitorFunc) const = 0;
	virtual void forEachIntersectingItem(const mRay3 &ray, const RayIntersectionTestFunc3<T> &testFunc, const IntersectionVisitorFunc3<T> &visitorFunc) const = 0;
	virtual auto closestIntersection(const mRay3 &ray, const GetRayIntersectionFunc3<T> &testFunc) const -> ClosestIntersectionQueryResult3<T> = 0;
};
} // namespace HinaPE::Util

#endif //HINAPE_INTERSECTION_QUERY_ENGINE_H
