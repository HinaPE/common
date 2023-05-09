#ifndef HINAPE_NEAREST_NEIGHBOR_QUERY_ENGINE_H
#define HINAPE_NEAREST_NEIGHBOR_QUERY_ENGINE_H

#include "math/vector.h"

namespace HinaPE::Util
{

template<typename T>
struct NearestNeighborQueryResult3
{
	const T *item = nullptr;
	real distance = Constant::I_REAL_MAX;
};

template<typename T> using NearestNeighborDistanceFunc3 = std::function<real(const T &, const mVector3 &)>;

template<typename T>
class NearestNeighborQueryEngine3
{
public:
	virtual auto nearest(const mVector3 &pt, const NearestNeighborDistanceFunc3<T> &distanceFunc) const -> NearestNeighborQueryResult3<T> = 0;
};
} // namespace HinaPE::Util

#endif //HINAPE_NEAREST_NEIGHBOR_QUERY_ENGINE_H
