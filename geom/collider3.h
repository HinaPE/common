#ifndef HINAPE_COLLIDER3_H
#define HINAPE_COLLIDER3_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "surface3.h"

namespace HinaPE::Geom
{
class Collider3
{
public:
	void load_surface(Surface3 *surface) { _surface = surface; }
	void resolve_collision(real radius, real restitution, mVector3 &position, mVector3 &velocity) const;
	void update(real current_time, real time_interval);
	virtual auto velocity_at(const mVector3 &point) const -> mVector3 = 0;

public:
	struct Opt
	{
		real friction = Constant::Zero;
	} _opt;
	struct ColliderQueryResult final
	{
		real distance;
		mVector3 point;
		mVector3 normal;
		mVector3 velocity;
	};

protected:
	auto get_closest_point(Surface3 *surface, const mVector3 &query_point) const -> ColliderQueryResult;
	auto is_penetrating(const ColliderQueryResult &result, const mVector3 &position, real radius) const -> bool;

protected:
	Surface3 *_surface;
	using OnBeginUpdateCallback = std::function<void(Collider3 *, real, real)>;
	OnBeginUpdateCallback _on_begin_update_callback;
};
using Collider3Ptr = std::shared_ptr<Collider3>;


class RigidBodyCollider3 : public Collider3
{
public:
	auto velocity_at(const mVector3 &point) const -> mVector3 final;

public:
//	explicit RigidBodyCollider3(Surface3Ptr surface) : Collider3(std::move(surface)) {}
//	explicit RigidBodyCollider3(Surface3Ptr surface, mVector3 linear_velocity, mVector3 angular_velocity) : Collider3(std::move(surface)), _linear_velocity(std::move(linear_velocity)), _angular_velocity(std::move(angular_velocity)) {}

private:
	mVector3 _linear_velocity;
	mVector3 _angular_velocity;
};
using RigidBodyCollider3Ptr = std::shared_ptr<RigidBodyCollider3>;
}

#endif //HINAPE_COLLIDER3_H
