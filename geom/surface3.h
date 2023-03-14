#ifndef HINAPE_SURFACE3_H
#define HINAPE_SURFACE3_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "math/vector.h"
#include "math/ray.h"
#include "math/transform.h"
#include "geom/bbox.h"

namespace HinaPE::Geom
{
struct SurfaceRayIntersection3
{
	bool is_intersecting = false;
	real distance = std::numeric_limits<real>::max();
	mVector3 point;
	mVector3 normal;
	unsigned int ID = std::numeric_limits<unsigned int>::max();
	unsigned int particleID = std::numeric_limits<unsigned int>::max(); // only for particle objects
};

// @formatter:off
class Surface3
{
public:
	void flip_normal() { _opt.normal_flipped = true; } // flip normal to enable inside collider

public:
	virtual void update_query_engine() {}
	virtual auto is_bounded() -> bool 											{ return true; }
	virtual auto is_valid_geometry() -> bool 									{ return true; }
	inline auto intersects(const mRay3 &ray) const -> bool 						{ return _intersects_local(_transform.to_local(ray)); }
	inline auto bounding_box() const -> mBBox3 									{ return _transform.to_world(_bounding_box_local()); }
	inline auto closest_point(const mVector3 &other_point) const -> mVector3 	{ return _transform.to_world(_closest_point_local(_transform.to_local(other_point))); }
	inline auto closest_distance(const mVector3 &other_point) const -> real 	{ return _closest_distance_local(_transform.to_local(other_point)); }
	inline auto closest_normal(const mVector3 &other_point) const -> mVector3 	{ return ((_opt.normal_flipped) ? -Constant::One : Constant::One) * _transform.to_world_direction(_closest_normal_local(_transform.to_local(other_point))); }
	inline auto is_inside(const mVector3 &point) -> bool 						{ return _opt.normal_flipped == !_is_inside_local(_transform.to_local(point)); }

public:
	struct Opt
	{
		bool normal_flipped = false;
	} _opt;

public:
	explicit Surface3(mTransform3 transform = mTransform3()) : _transform(std::move(transform)) {}

protected:
	virtual auto _closest_point_local(const mVector3 &other_point) const -> mVector3 = 0;
	virtual auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 = 0;
	virtual auto _closest_distance_local(const mVector3 &other_point) const -> real;
	virtual auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 = 0;
	virtual auto _is_inside_local(const mVector3 &other_point) const -> bool;
	virtual auto _intersects_local(const mRay3 &ray) const -> bool = 0;
	virtual auto _bounding_box_local() const -> mBBox3 = 0;

public:
	mTransform3 _transform;
};
// @formatter:on

class Box3 : public Surface3
{
public:
	mBBox3 _bound;
	mVector3 _extent;

protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;
};

class Sphere3 : public Surface3
{
public:
	mVector3 _center;
	real _radius = 1;

protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;
};

class Plane3 : public Surface3
{
public:
	mVector3 _point;
	mVector3 _normal;
	explicit Plane3(mVector3 point = mVector3::Zero(), mVector3 normal = mVector3::UnitY());

protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;
};

class Cylinder3 : public Surface3
{
protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;
};

class ImplicitSurface3 : public Surface3
{
public:
	auto signed_distance(const mVector3 &other_point) const -> real { return _opt.normal_flipped ? -_signed_distance_local(_transform.to_local(other_point)) : _signed_distance_local(_transform.to_local(other_point)); }

protected:
	auto _closest_distance_local(const mVector3 &other_point) const -> real final { return std::fabs(_signed_distance_local(other_point)); }
	auto _is_inside_local(const mVector3 &other_point) const -> bool final { return Math::is_inside_sdf(_signed_distance_local(other_point)); }

	virtual auto _signed_distance_local(const mVector3 &other_point) const -> real = 0;
};

class SurfaceToImplicit3 : public ImplicitSurface3
{
public:
	explicit SurfaceToImplicit3(const std::shared_ptr<Surface3> &surface);

protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _signed_distance_local(const mVector3 &other_point) const -> real final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;

private:
	std::shared_ptr<Surface3> _surface;
};

//@formatter:off
using Surface3Ptr           = std::shared_ptr<Surface3>;
using Box3Ptr               = std::shared_ptr<Box3>;
using Sphere3Ptr            = std::shared_ptr<Sphere3>;
using ImplicitSurface3Ptr   = std::shared_ptr<ImplicitSurface3>;
using SurfaceToImplicit3Ptr = std::shared_ptr<SurfaceToImplicit3>;
//@formatter:on
} // namespace HinaPE::Geom

#endif //HINAPE_SURFACE3_H
