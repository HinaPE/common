#ifndef HINAPE_COMMON_H
#define HINAPE_COMMON_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "math/array.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "math/quaternion.h"
#include "math/ray.h"
#include "math/transform.h"
#include "math/time_integration.h"
#include "geom/bbox.h"
#include "geom/surface3.h"
#include "util/parallel.h"

namespace HinaPE
{
class CopyDisable
{
public:
	CopyDisable() = default;
	~CopyDisable() = default;
	CopyDisable(const CopyDisable &) = delete;
	auto operator=(const CopyDisable &) -> CopyDisable & = delete;
	CopyDisable(CopyDisable &&) = default;
	auto operator=(CopyDisable &&) -> CopyDisable & = default;
};
}  // namespace HinaPE
// @formatter:off
template<typename T> auto is(const auto *src) -> bool { return dynamic_cast<const T *>(src) != nullptr; }
template<typename T> auto as(auto *src) -> T * { return dynamic_cast<T *>(src); }
// @formatter:on

#endif //HINAPE_COMMON_H
