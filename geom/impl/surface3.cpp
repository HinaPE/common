#include <utility>

#include "geom/surface3.h"

auto HinaPE::Geom::Surface3::_closest_distance_local(const mVector3 &other_point) const -> real { return (other_point - _closest_point_local(other_point)).length(); }
auto HinaPE::Geom::Surface3::_is_inside_local(const mVector3 &other_point) const -> bool { return (other_point - _closest_point_local(other_point)).dot(_closest_normal_local(other_point)) < 0; }

// ============================== Box ==============================
auto HinaPE::Geom::Box3::_intersects_local(const mRay3 &ray) const -> bool
{
	real t_min = Constant::Zero;
	real t_max = Constant::Infinity;
	const mVector3 ray_inv_dir = ray._direction.reciprocal();

	for (int i = 0; i < 3; ++i)
	{
		real t_near = (_bound._lower_corner[i] - ray._origin[i]) * ray_inv_dir[i];
		real t_far = (_bound._upper_corner[i] - ray._origin[i]) * ray_inv_dir[i];

		if (t_near > t_far) std::swap(t_near, t_far);
		t_min = std::max(t_min, t_near);
		t_max = std::min(t_max, t_far);

		if (t_min > t_max) return false;
	}
	return true;
}
auto HinaPE::Geom::Box3::_bounding_box_local() const -> mBBox3
{
	return _bound;
}
auto HinaPE::Geom::Box3::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	if (_bound.contains(other_point))
	{
		std::array<Plane3, 6> planes = {
				Plane3(_bound._upper_corner, mVector3(1, 0, 0)),
				Plane3(_bound._upper_corner, mVector3(0, 1, 0)),
				Plane3(_bound._upper_corner, mVector3(0, 0, 1)),
				Plane3(_bound._lower_corner, mVector3(-1, 0, 0)),
				Plane3(_bound._lower_corner, mVector3(0, -1, 0)),
				Plane3(_bound._lower_corner, mVector3(0, 0, -1))
		};

		mVector3 res = planes[0].closest_point(other_point);
		real distance_squared = (res - other_point).length_squared();

		for (int i = 1; i < 6; ++i)
		{
			mVector3 local_res = planes[i].closest_point(other_point);
			real local_distance_squared = (local_res - other_point).length_squared();

			if (local_distance_squared < distance_squared)
			{
				res = local_res;
				distance_squared = local_distance_squared;
			}
		}
		return res;
	} else
		return clamp(other_point, _bound._lower_corner, _bound._upper_corner);
}
auto HinaPE::Geom::Box3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	SurfaceRayIntersection3 res;

	real t_min = Constant::Zero;
	real t_max = Constant::Infinity;
	const mVector3 ray_inv_dir = ray._direction.reciprocal();

	for (int i = 0; i < 3; ++i)
	{
		real t_near = (_bound._lower_corner[i] - ray._origin[i]) * ray_inv_dir[i];
		real t_far = (_bound._upper_corner[i] - ray._origin[i]) * ray_inv_dir[i];

		if (t_near > t_far) std::swap(t_near, t_far);
		t_min = std::max(t_min, t_near);
		t_max = std::min(t_max, t_far);

		if (t_min > t_max)
		{
			res.is_intersecting = false;
			return res;
		}
	}

	res.is_intersecting = true;

	if (_bound.contains(ray._origin))
	{
		res.distance = t_min;
		res.point = ray.point_at(t_min);
		res.normal = _closest_normal_local(res.point);
	} else
	{
		res.distance = t_max;
		res.point = ray.point_at(t_max);
		res.normal = _closest_normal_local(res.point);
	}

	return res;
}
auto HinaPE::Geom::Box3::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	std::array<Plane3, 6> planes = {
			Plane3(_bound._upper_corner, mVector3(1, 0, 0)),
			Plane3(_bound._upper_corner, mVector3(0, 1, 0)),
			Plane3(_bound._upper_corner, mVector3(0, 0, 1)),
			Plane3(_bound._lower_corner, mVector3(-1, 0, 0)),
			Plane3(_bound._lower_corner, mVector3(0, -1, 0)),
			Plane3(_bound._lower_corner, mVector3(0, 0, -1))
	};
	if (_bound.contains(other_point))
	{
		mVector3 closest_point = planes[0].closest_point(other_point);
		mVector3 closest_normal = planes[0]._normal;
		real min_distance_squared = (closest_point - other_point).length_squared();

		for (int i = 1; i < 6; ++i)
		{
			mVector3 local_closest_point = planes[i].closest_point(other_point);
			real local_distance_squared = (local_closest_point - other_point).length_squared();

			if (local_distance_squared < min_distance_squared)
			{
				closest_normal = planes[i]._normal;
				min_distance_squared = local_distance_squared;
			}
		}
		return closest_normal;
	} else
	{
		mVector3 closest_point = clamp(other_point, _bound._lower_corner, _bound._upper_corner);
		mVector3 closest_point_to_input_point = other_point - closest_point;
		mVector3 closest_normal = planes[0]._normal;
		real max_cosine_angle = closest_normal.dot(closest_point_to_input_point);

		for (int i = 1; i < 6; ++i)
		{
			real cosine_angle = planes[i]._normal.dot(closest_point_to_input_point);
			if (cosine_angle > max_cosine_angle)
			{
				closest_normal = planes[i]._normal;
				max_cosine_angle = cosine_angle;
			}
		}

		return closest_normal;
	}
}

// ============================== Sphere ==============================
auto HinaPE::Geom::Sphere3::_closest_point_local(const mVector3 &other_point) const -> mVector3 { return _radius * _closest_normal_local(other_point) + _center; }
auto HinaPE::Geom::Sphere3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	HinaPE::Geom::SurfaceRayIntersection3 res;
	mVector3 r = ray._origin - _center;
	real b = r.dot(ray._direction);
	real c = r.dot(r) - _radius * _radius;
	real d = b * b - c;

	if (d > Constant::Zero)
	{
		d = std::sqrt(d);
		real t_min = -b - d;
		real t_max = -b + d;

		if (t_min < Constant::Zero)
			t_min = t_max;

		if (t_min < 0)
			res.is_intersecting = false;
		else
		{
			res.is_intersecting = true;
			res.distance = t_min;
			res.point = ray.point_at(t_min);
			res.point_far = ray.point_at(t_max);
			res.normal = (res.point - _center).normalized();
		}
	} else
		res.is_intersecting = false;
	return res;
}
auto HinaPE::Geom::Sphere3::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	if (similar(_center, other_point))
		return mVector3::UnitY();
	else
		return (other_point - _center).normalized();
}
auto HinaPE::Geom::Sphere3::_intersects_local(const mRay3 &ray) const -> bool
{
	mVector3 r = ray._origin - _center;
	real b = r.dot(ray._direction);
	real c = r.dot(r) - _radius * _radius;
	real d = b * b - c;

	if (d > Constant::Zero)
	{
		d = std::sqrt(d);
		real t_min = -b - d;
		real t_max = -b + d;

		if (t_min < Constant::Zero)
			t_min = t_max;

		if (t_min > 0)
			return true;
	}
	return false;
}
auto HinaPE::Geom::Sphere3::_bounding_box_local() const -> mBBox3 { return {_center - mVector3(_radius, _radius, _radius), _center + mVector3(_radius, _radius, _radius)}; }

// ============================== Plane ==============================
HinaPE::Geom::Plane3::Plane3(mVector3 point, mVector3 normal) : _point(std::move(point)), _normal(std::move(normal)) {}
auto HinaPE::Geom::Plane3::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	mVector3 r = other_point - _point;
	mVector3 t = _normal.dot(r) * _normal + _point;
	return r - _normal.dot(r) * _normal + _point;
}
auto HinaPE::Geom::Plane3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	SurfaceRayIntersection3 res;
	real d_dot_n = ray._direction.dot(_normal);

	if (std::fabs(d_dot_n) > Constant::Epsilon)
	{
		real t = _normal.dot(_point - ray._origin) / d_dot_n;
		if (t >= 0)
		{
			res.is_intersecting = true;
			res.distance = t;
			res.point = ray.point_at(t);
			res.normal = _normal;
		}
	}

	return res;
}
auto HinaPE::Geom::Plane3::_closest_normal_local(const mVector3 &other_point) const -> mVector3 { return _normal; }
auto HinaPE::Geom::Plane3::_intersects_local(const mRay3 &ray) const -> bool { return std::fabs(ray._direction.dot(_normal)) > 0; }
auto HinaPE::Geom::Plane3::_bounding_box_local() const -> mBBox3
{
	if (std::fabs(_normal.dot(mVector3(1, 0, 0)) - Constant::One) < Constant::Epsilon)
		return {_point - mVector3(-Constant::Zero, -Constant::Infinity, -Constant::Infinity),
				_point + mVector3(Constant::Zero, Constant::Infinity, Constant::Infinity)};
	else if (std::fabs(_normal.dot(mVector3(0, 1, 0)) - Constant::One) < Constant::Epsilon)
		return {_point - mVector3(-Constant::Infinity, -Constant::Zero, -Constant::Infinity),
				_point + mVector3(Constant::Infinity, Constant::Zero, Constant::Infinity)};
	else if (std::fabs(_normal.dot(mVector3(0, 0, 1)) - Constant::One) < Constant::Epsilon)
		return {_point - mVector3(-Constant::Infinity, -Constant::Infinity, -Constant::Zero),
				_point + mVector3(Constant::Infinity, Constant::Infinity, Constant::Zero)};
	else
		return {mVector3(Constant::Infinity, Constant::Infinity, Constant::Infinity),
				mVector3(Constant::Infinity, Constant::Infinity, Constant::Infinity)};
}


// ============================== Cylinder3 ==============================
auto HinaPE::Geom::Cylinder3::_intersects_local(const mRay3 &ray) const -> bool { return false; }
auto HinaPE::Geom::Cylinder3::_bounding_box_local() const -> mBBox3 { return {}; }
auto HinaPE::Geom::Cylinder3::_closest_point_local(const mVector3 &other_point) const -> mVector3 { return {}; }
auto HinaPE::Geom::Cylinder3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3 { return {}; }
auto HinaPE::Geom::Cylinder3::_closest_normal_local(const mVector3 &other_point) const -> mVector3 { return {}; }


// ============================== SurfaceToImplicit3 ==============================
HinaPE::Geom::SurfaceToImplicit3::SurfaceToImplicit3(const std::shared_ptr<Surface3> &surface) : _surface(surface) {}
auto HinaPE::Geom::SurfaceToImplicit3::is_bounded() -> bool { return _surface->is_bound(); }
void HinaPE::Geom::SurfaceToImplicit3::update_query_engine() { _surface->update_query_engine(); }
auto HinaPE::Geom::SurfaceToImplicit3::is_valid_geometry() -> bool { return _surface->is_valid_geometry(); }
auto HinaPE::Geom::SurfaceToImplicit3::_closest_point_local(const mVector3 &other_point) const -> mVector3 { return _surface->closest_point(other_point); }
auto HinaPE::Geom::SurfaceToImplicit3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3 { return _surface->closest_intersection(ray); }
auto HinaPE::Geom::SurfaceToImplicit3::_closest_normal_local(const mVector3 &other_point) const -> mVector3 { return _surface->closest_normal(other_point); }
auto HinaPE::Geom::SurfaceToImplicit3::_signed_distance_local(const mVector3 &other_point) const -> real
{
	mVector3 x = _surface->closest_point(other_point);
	bool inside = _surface->is_inside(other_point);
	real d = (x - other_point).length();
	return inside ? -d : d;
}
auto HinaPE::Geom::SurfaceToImplicit3::_intersects_local(const mRay3 &ray) const -> bool { return _surface->intersects(ray); }
auto HinaPE::Geom::SurfaceToImplicit3::_bounding_box_local() const -> mBBox3 { return _surface->bounding_box(); }
auto HinaPE::Geom::SurfaceToImplicit3::_closest_distance_local(const mVector3 &other_point) const -> real { return _surface->closest_distance(other_point); }
auto HinaPE::Geom::SurfaceToImplicit3::_is_inside_local(const mVector3 &other_point) const -> bool { return _surface->is_inside(other_point); }
