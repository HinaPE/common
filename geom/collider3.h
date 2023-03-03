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
	using OnBeginUpdateCallback = std::function<void(Collider3 *, real, real)>;
	void update(real current_time, real time_interval);
	void resolve_collision(real radius, real restitution, mVector3 &position, mVector3 &velocity) const;

	virtual auto velocity_at(const mVector3 &point) const -> mVector3 = 0;

public:
	struct Opt
	{
		real friction = Constant::Zero;
		OnBeginUpdateCallback _on_begin_update_callback;
	} _opt;
	struct ColliderQueryResult final
	{
		real distance;
		mVector3 point;
		mVector3 normal;
		mVector3 velocity;
	};

protected:
	auto get_closest_point(const Surface3Ptr &surface, const mVector3 &query_point) const -> ColliderQueryResult;
	auto is_penetrating(const ColliderQueryResult &result, const mVector3 &position, real radius) const -> bool;

private:
	Surface3Ptr _surface;
};
using Collider3Ptr = std::shared_ptr<Collider3>;


class RigidBodyCollider3 final : public Collider3
{
public:
	auto velocity_at(const mVector3 &point) const -> mVector3 final;

};
using RigidBodyCollider3Ptr = std::shared_ptr<RigidBodyCollider3>;
}

#endif //HINAPE_COLLIDER3_H
