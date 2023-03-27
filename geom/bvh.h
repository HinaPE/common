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
	auto _build(size_t node_index, size_t *item_indices, size_t n_items, size_t current_depth) -> size_t;
	auto _qsplit(size_t *item_indices, size_t num_items, real pivot, uint8_t axis) -> size_t;

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

	_build(0, item_indices.data(), size, 0);
}

template<typename T>
auto BVH<T>::_build(size_t node_index, size_t *item_indices, size_t n_items, size_t current_depth) -> size_t
{
	_nodes.push_back(Node());

	if (n_items == 1)
	{
		_nodes[node_index].init_leaf(item_indices[0], _item_bounds[item_indices[0]]);
		return current_depth + 1;
	}

	mBBox3 node_bound;
	for (size_t i = 0; i < n_items; ++i)
		node_bound.merge(_item_bounds[item_indices[i]]);

	mVector3 d = node_bound._upper_corner - node_bound._lower_corner;

	uint8_t axis = 0;
	if (d.x() > d.y() && d.x() > d.z())
		axis = 0;
	else
		axis = d.y() > d.z() ? 1 : 2;

	real pivot = 0.5f * (node_bound._lower_corner[axis] + node_bound._upper_corner[axis]);

	// classify primitives with respect to split
	size_t mid_point = _qsplit(item_indices, n_items, pivot, axis);

	// recursively initialize children _nodes
	size_t d0 = _build(node_index + 1, item_indices, mid_point, current_depth + 1);
	_nodes[node_index].init_internal(axis, _nodes.size(), node_bound);
	size_t d1 = _build(_nodes[node_index].child, item_indices + mid_point, n_items - mid_point, current_depth + 1);

	return std::max(d0, d1);
}

template<typename T>
auto BVH<T>::_qsplit(size_t *item_indices, size_t num_items, real pivot, uint8_t axis) -> size_t
{
	real centroid;
	size_t ret = 0;

	for (int i = 0; i < num_items; ++i)
	{
		mBBox3 b = _item_bounds[item_indices[i]];
		centroid = 0.5f * (b._lower_corner[axis] + b._upper_corner[axis]);
		if (centroid < pivot)
		{
			std::swap(item_indices[i], item_indices[ret]);
			++ret;
		}
	}
	if (ret == 0 || ret == num_items)
		ret = num_items >> 1;

	return ret;
}

// @formatter:off
template<typename T> BVH<T>::Node::Node() : child(Constant::I_SIZE_MAX), bound() {}
template<typename T> void BVH<T>::Node::init_leaf(size_t it, const mBBox3 &b) { flags = 3; item = it; bound = b; }
template<typename T> void BVH<T>::Node::init_internal(uint8_t axis, size_t c, const mBBox3 &b) { flags = axis; child = c; bound = b; }
template<typename T> auto BVH<T>::Node::is_leaf() const -> bool { return flags == 3; }
// @formatter:on
} // namespace HinaPE::Geom

#endif //HINAPE_BVH_H
