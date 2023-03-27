#ifndef HINAPE_BVH_H
#define HINAPE_BVH_H

#include "bbox.h"

namespace HinaPE::Geom
{
template<typename T>
class BVH
{
public:
	void build(const std::vector<T> &items, const std::vector<mBBox3> &items_bounds);

public:
	struct Node
	{
		uint8_t flags{0};
		union
		{
			size_t child;
			size_t item;
		};
		mBBox3 bound;

		Node();
		void init_leaf(size_t it, const mBBox3 &b);
		void init_internal(uint8_t axis, size_t c, const mBBox3 &b);
		auto is_leaf() const -> bool;
	};

private:
	// @formatter:off
	mBBox3 				_bound;
	std::vector<T> 		_items;
	std::vector<mBBox3> _item_bounds;
	std::vector<Node> 	_nodes;
	// @formatter:on
};

template<typename T>
void BVH<T>::build(const std::vector<T> &items, const std::vector<mBBox3> &items_bounds)
{
	_items = items;
	_item_bounds = items_bounds;

	if (_items.empty())
		return;

	auto size = _items.size();

	_nodes.clear();
	_bound = mBBox3();

	for (auto &bound: _item_bounds)
		_bound.merge(bound);

	std::vector<size_t> item_indices(size);
	std::iota(item_indices.begin(), item_indices.end(), 0);
}

// @formatter:off
template<typename T> BVH<T>::Node::Node() : child(Constant::I_SIZE_MAX), bound() {}
template<typename T> void BVH<T>::Node::init_leaf(size_t it, const mBBox3 &b) { flags = 3; item = it; bound = b; }
template<typename T> void BVH<T>::Node::init_internal(uint8_t axis, size_t c, const mBBox3 &b) { flags = axis; child = c; bound = b; }
template<typename T> auto BVH<T>::Node::is_leaf() const -> bool { return flags == 3; }
// @formatter:on
} // namespace HinaPE::Geom

#endif //HINAPE_BVH_H
