#include "geom/bvh.h"

HinaPE::Geom::BVH::Node::Node() : flags(0), child(Constant::I_SIZE_MAX), bound() {}
void HinaPE::Geom::BVH::Node::init_leaf(size_t it, const mBBox3 &b)
{
	flags = 3;
	item = it;
	bound = b;
}
void HinaPE::Geom::BVH::Node::init_internal(uint8_t axis, size_t c, const mBBox3 &b)
{
	flags = axis;
	child = c;
	bound = b;
}
auto HinaPE::Geom::BVH::Node::is_leaf() const -> bool { return flags == 3; }
