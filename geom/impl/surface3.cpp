#include "geom/surface3.h"

// ============================== Box ==============================
auto HinaPE::Geom::Box3::_intersects_local(const mRay3 &ray) const -> bool
{
	real t_min = 0;
	real t_max = std::numeric_limits<real>::max();
	const mVector3 ray_inv_dir = ray._direction.reciprocal();
	return false;
}
auto HinaPE::Geom::Box3::_bounding_box_local() const -> mBBox3
{
	return _bound;
}
auto HinaPE::Geom::Box3::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	if (_bound.contains(other_point))
	{
	}
	return {};
}
auto HinaPE::Geom::Box3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return {};
}
auto HinaPE::Geom::Box3::_closest_distance_local(const mVector3 &other_point) const -> real
{
	return 0;
}
auto HinaPE::Geom::Box3::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}

// ============================== Sphere ==============================
auto HinaPE::Geom::Sphere3::_intersects_local(const mRay3 &ray) const -> bool
{
	return false;
}
auto HinaPE::Geom::Sphere3::_bounding_box_local() const -> mBBox3
{
	return {};
}
auto HinaPE::Geom::Sphere3::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}
auto HinaPE::Geom::Sphere3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return {};
}
auto HinaPE::Geom::Sphere3::_closest_distance_local(const mVector3 &other_point) const -> real
{
	return 0;
}
auto HinaPE::Geom::Sphere3::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}

// ============================== Plane ==============================
auto HinaPE::Geom::Plane3::_intersects_local(const mRay3 &ray) const -> bool
{
	return std::fabs(ray._direction.dot(_normal)) > 0.0f;
}
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
auto HinaPE::Geom::Plane3::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	mVector3 r = other_point - _point;
	return _point + _normal * r.dot(_normal);
}
auto HinaPE::Geom::Plane3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return {};
}
auto HinaPE::Geom::Plane3::_closest_distance_local(const mVector3 &other_point) const -> real
{
	return 0;
}
auto HinaPE::Geom::Plane3::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}


// ============================== Cylinder3 ==============================
auto HinaPE::Geom::Cylinder3::_intersects_local(const mRay3 &ray) const -> bool
{
	return false;
}
auto HinaPE::Geom::Cylinder3::_bounding_box_local() const -> mBBox3
{
	return {};
}
auto HinaPE::Geom::Cylinder3::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}
auto HinaPE::Geom::Cylinder3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return {};
}
auto HinaPE::Geom::Cylinder3::_closest_distance_local(const mVector3 &other_point) const -> real
{
	return 0;
}
auto HinaPE::Geom::Cylinder3::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}


// ============================== SurfaceToImplicit3 ==============================
HinaPE::Geom::SurfaceToImplicit3::SurfaceToImplicit3(const std::shared_ptr<Surface3> &surface) : _surface(surface)
{
}
auto HinaPE::Geom::SurfaceToImplicit3::_intersects_local(const mRay3 &ray) const -> bool
{
	return false;
}
auto HinaPE::Geom::SurfaceToImplicit3::_bounding_box_local() const -> mBBox3
{
	return {};
}
auto HinaPE::Geom::SurfaceToImplicit3::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}
auto HinaPE::Geom::SurfaceToImplicit3::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return {};
}
auto HinaPE::Geom::SurfaceToImplicit3::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return {};
}
auto HinaPE::Geom::SurfaceToImplicit3::_signed_distance_local(const mVector3 &other_point) const -> real
{
	return 0;
}
