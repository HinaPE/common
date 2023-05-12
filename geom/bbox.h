#ifndef HINAPE_BBOX_H
#define HINAPE_BBOX_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "math/vector.h"
#include "math/ray.h"

namespace HinaPE::Geom
{
template<typename T>
struct BoundingBoxRayIntersection3
{
	bool is_intersecting = false;
	T t_near = std::numeric_limits<T>::max();
	T t_far = std::numeric_limits<T>::max();
};

template<typename T>
class BoundingBox3
{
public:
	auto width() const -> T;
	auto height() const -> T;
	auto depth() const -> T;
	auto length(size_t axis) -> T;
	auto overlaps(const BoundingBox3 &other) const -> bool;
	auto contains(const Math::Vector3<T> &point) const -> bool;
	auto corner(size_t idx) const -> Math::Vector3<T>;
	auto center() const -> Math::Vector3<T>;
	void merge(const Math::Vector3<T> &point);
	void merge(const BoundingBox3 &bbox);
	void expand(T delta);
	void reset();
	auto clamp(const Math::Vector3<T> &point) const -> Math::Vector3<T>;
	auto intersects(const Math::Ray3<T> &ray) const -> bool;

public:
	BoundingBox3();
	explicit BoundingBox3(const std::vector<mVector3> &points);
	BoundingBox3(const Math::Vector3<T> &point1, const Math::Vector3<T> &point2);
	BoundingBox3(const BoundingBox3 &bbox) = default;
	BoundingBox3(BoundingBox3 &&bbox) noexcept = default;
	auto operator=(const BoundingBox3 &bbox) -> BoundingBox3 & = default;
	auto operator=(BoundingBox3 &&bbox) noexcept -> BoundingBox3 & = default;

public:
	Math::Vector3<T> _lower_corner, _upper_corner;
};

template<typename T>
BoundingBox3<T>::BoundingBox3() : _lower_corner(std::numeric_limits<T>::max()), _upper_corner(-std::numeric_limits<T>::max()) {}
template<typename T>
BoundingBox3<T>::BoundingBox3(const std::vector<mVector3> &points) : BoundingBox3() { for (const auto &point: points) merge(point); }
template<typename T>
BoundingBox3<T>::BoundingBox3(const Math::Vector3<T> &point1, const Math::Vector3<T> &point2)
{
	_lower_corner.x() = std::min(point1.x(), point2.x());
	_lower_corner.y() = std::min(point1.y(), point2.y());
	_lower_corner.z() = std::min(point1.z(), point2.z());
	_upper_corner.x() = std::max(point1.x(), point2.x());
	_upper_corner.y() = std::max(point1.y(), point2.y());
	_upper_corner.z() = std::max(point1.z(), point2.z());
}

template<typename T>
auto BoundingBox3<T>::width() const -> T { return _upper_corner.x() - _lower_corner.x(); }
template<typename T>
auto BoundingBox3<T>::height() const -> T { return _upper_corner.y() - _lower_corner.y(); }
template<typename T>
auto BoundingBox3<T>::depth() const -> T { return _upper_corner.z() - _lower_corner.z(); }
template<typename T>
auto BoundingBox3<T>::length(size_t axis) -> T
{
	assert(axis < 3);
	return _upper_corner[axis] - _lower_corner[axis];
}
template<typename T>
auto BoundingBox3<T>::overlaps(const BoundingBox3 &other) const -> bool
{
	if (_upper_corner.x() < other._lower_corner.x() || _lower_corner.x() > other._upper_corner.x()) return false;
	if (_upper_corner.y() < other._lower_corner.y() || _lower_corner.y() > other._upper_corner.y()) return false;
	if (_upper_corner.z() < other._lower_corner.z() || _lower_corner.z() > other._upper_corner.z()) return false;
	return true;
}
template<typename T>
auto BoundingBox3<T>::contains(const Math::Vector3<T> &point) const -> bool
{
	if (_lower_corner.x() > point.x() || _upper_corner.x() < point.x()) return false;
	if (_lower_corner.y() > point.y() || _upper_corner.y() < point.y()) return false;
	if (_lower_corner.z() > point.z() || _upper_corner.z() < point.z()) return false;
	return true;
}
template<typename T>
auto BoundingBox3<T>::corner(size_t idx) const -> Math::Vector3<T>
{
	static const T h = static_cast<T>(1) / 2;
	static const std::array<Math::Vector3<T>, 8> offset = {
			Math::Vector3<T>(-h, -h, -h),
			Math::Vector3<T>(+h, -h, -h),
			Math::Vector3<T>(-h, +h, -h),
			Math::Vector3<T>(+h, +h, -h),
			Math::Vector3<T>(-h, -h, +h),
			Math::Vector3<T>(+h, -h, +h),
			Math::Vector3<T>(-h, +h, +h),
			Math::Vector3<T>(+h, +h, +h)
	};

	return Math::Vector3<T>(width(), height(), depth()) * offset[idx] + center();
}
template<typename T>
auto BoundingBox3<T>::center() const -> Math::Vector3<T> { return (_lower_corner + _upper_corner) / static_cast<T>(2); }
template<typename T>
void BoundingBox3<T>::merge(const Math::Vector3<T> &point)
{
	_lower_corner.x() = std::min(_lower_corner.x(), point.x());
	_lower_corner.y() = std::min(_lower_corner.y(), point.y());
	_lower_corner.z() = std::min(_lower_corner.z(), point.z());
	_upper_corner.x() = std::max(_upper_corner.x(), point.x());
	_upper_corner.y() = std::max(_upper_corner.y(), point.y());
	_upper_corner.z() = std::max(_upper_corner.z(), point.z());
}
template<typename T>
void BoundingBox3<T>::merge(const BoundingBox3 &bbox)
{
	_lower_corner.x() = std::min(_lower_corner.x(), bbox._lower_corner.x());
	_lower_corner.y() = std::min(_lower_corner.y(), bbox._lower_corner.y());
	_lower_corner.z() = std::min(_lower_corner.z(), bbox._lower_corner.z());
	_upper_corner.x() = std::max(_upper_corner.x(), bbox._upper_corner.x());
	_upper_corner.y() = std::max(_upper_corner.y(), bbox._upper_corner.y());
	_upper_corner.z() = std::max(_upper_corner.z(), bbox._upper_corner.z());
}
template<typename T>
void BoundingBox3<T>::expand(T delta)
{
	_lower_corner.x() -= delta;
	_lower_corner.y() -= delta;
	_lower_corner.z() -= delta;
	_upper_corner.x() += delta;
	_upper_corner.y() += delta;
	_upper_corner.z() += delta;
}
template<typename T>
void BoundingBox3<T>::reset()
{
	_lower_corner.x() = std::numeric_limits<T>::max();
	_lower_corner.y() = std::numeric_limits<T>::max();
	_lower_corner.z() = std::numeric_limits<T>::max();
	_upper_corner.x() = -std::numeric_limits<T>::max();
	_upper_corner.y() = -std::numeric_limits<T>::max();
	_upper_corner.z() = -std::numeric_limits<T>::max();
}
template<typename T>
auto BoundingBox3<T>::clamp(const Math::Vector3<T> &point) const -> Math::Vector3<T>
{
	return {
			std::clamp(point.x(), _lower_corner.x(), _upper_corner.x()),
			std::clamp(point.y(), _lower_corner.y(), _upper_corner.y()),
			std::clamp(point.z(), _lower_corner.z(), _upper_corner.z())
	};
}
template<typename T>
auto BoundingBox3<T>::intersects(const Math::Ray3<T> &ray) const -> bool
{
	T min = 0, max = std::numeric_limits<T>::max();
	const Math::Vector3<T> &ray_inv_dir = ray._direction.reciprocal();

	for (size_t i = 0; i < 3; ++i)
	{
		T t_near = (_lower_corner[i] - ray._origin[i]) * ray_inv_dir[i];
		T t_far = (_upper_corner[i] - ray._origin[i]) * ray_inv_dir[i];

		if (t_near > t_far) std::swap(t_near, t_far);
		min = std::max(min, t_near);
		max = std::min(max, t_far);

		if (min > max) return false;
	}

	return true;
}
}

#ifdef HINAPE_DOUBLE
using mBBox3 = HinaPE::Geom::BoundingBox3<double>;
#else
using mBBox3 = HinaPE::Geom::BoundingBox3<float>;
#endif
using mBBox3i = HinaPE::Geom::BoundingBox3<int>;
#endif //HINAPE_BBOX_H
