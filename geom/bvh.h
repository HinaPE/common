#ifndef HINAPE_BVH_H
#define HINAPE_BVH_H

#include "bbox.h"
#include "util/intersection_query_engine.h"
#include "util/nearest_neighbor_query_engine.h"

namespace HinaPE::Geom
{
template<typename T>
class BVH final : public Util::IntersectionQueryEngine3<T>, public Util::NearestNeighborQueryEngine3<T>
{
public:
	void build(const std::vector<T> &items, const std::vector<mBBox3> &items_bounds);

public:
	auto intersects(const mBBox3 &box, const Util::BoxIntersectionTestFunc3<T> &testFunc) const -> bool final;
	auto intersects(const mRay3 &ray, const Util::RayIntersectionTestFunc3<T> &testFunc) const -> bool final;
	void forEachIntersectingItem(const mBBox3 &box, const Util::BoxIntersectionTestFunc3<T> &testFunc, const Util::IntersectionVisitorFunc3<T> &visitorFunc) const final;
	void forEachIntersectingItem(const mRay3 &ray, const Util::RayIntersectionTestFunc3<T> &testFunc, const Util::IntersectionVisitorFunc3<T> &visitorFunc) const final;
	auto closestIntersection(const mRay3 &ray, const Util::GetRayIntersectionFunc3<T> &testFunc) const -> Util::ClosestIntersectionQueryResult3<T> final;

public:
	auto nearest(const mVector3 &pt, const Util::NearestNeighborDistanceFunc3<T> &distanceFunc) const -> Util::NearestNeighborQueryResult3<T> final;

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
auto BVH<T>::intersects(const mBBox3 &box, const Util::BoxIntersectionTestFunc3<T> &testFunc) const -> bool
{
	if (!_bound.overlaps(box))
		return false;

	// prepare to traverse BVH for box
	static constexpr size_t MAX_TREE_DEPTH = 8 * sizeof(size_t);
	std::array<const Node *, MAX_TREE_DEPTH> todo;
	size_t todo_pos = 0;

	// traverse BVH nodes for box
	const Node *node = _nodes.data();

	while (node != nullptr)
	{
		if (node->is_leaf())
		{
			if (testFunc(_items[node->item], box))
				return true;

			// grab next node to process from todo stack
			if (todo_pos > 0)
			{
				node = todo[--todo_pos];
			} else
				break;
		} else
		{
			const Node *left = node + 1;
			const Node *right = &_nodes[node->child];

			// advance to next child node, possibly enqueue other child
			if (!left->bound.overlaps(box))
				node = right;
			else if (!right->bound.overlaps(box))
				node = left;
			else
			{
				// enqueue right child and continue with left
				todo[todo_pos++] = right;
				node = left;
			}
		}
	}

	return false;
}

template<typename T>
auto BVH<T>::intersects(const mRay3 &ray, const Util::RayIntersectionTestFunc3<T> &testFunc) const -> bool
{
	if (!_bound.intersects(ray))
		return false;

	// prepare to traverse BVH for ray
	static constexpr size_t MAX_TREE_DEPTH = 8 * sizeof(size_t);
	std::array<const Node *, MAX_TREE_DEPTH> todo;
	size_t todo_pos = 0;

	// traverse BVH nodes for ray
	const Node *node = _nodes.data();

	while (node != nullptr)
	{
		if (node->is_leaf())
		{
			if (testFunc(_items[node->item], ray))
				return true;

			// grab next node to process from todo stack
			if (todo_pos > 0)
			{
				node = todo[--todo_pos];
			} else
				break;
		} else
		{
			const Node *left;
			const Node *right;

			if (ray._direction[node->flags] > 0.0)
			{
				left = node + 1;
				right = (Node *) &_nodes[node->child];
			} else
			{
				left = (Node *) &_nodes[node->child];
				right = node + 1;
			}

			// advance to next child node, possibly enqueue other child
			if (!left->bound.intersects(ray))
				node = right;
			else if (!right->bound.intersects(ray))
				node = left;
			else
			{
				// enqueue secondChild in todo stack
				todo[todo_pos++] = right;
				node = left;
			}
		}
	}

	return false;
}

template<typename T>
void BVH<T>::forEachIntersectingItem(const mBBox3 &box, const Util::BoxIntersectionTestFunc3<T> &testFunc, const Util::IntersectionVisitorFunc3<T> &visitorFunc) const
{
	if (!_bound.overlaps(box))
		return;

	// prepare to traverse BVH for box
	static constexpr size_t MAX_TREE_DEPTH = 8 * sizeof(size_t);
	std::array<const Node *, MAX_TREE_DEPTH> todo;
	size_t todo_pos = 0;

	// traverse BVH nodes for box
	const Node *node = _nodes.data();

	while (node != nullptr)
	{
		if (node->is_leaf())
		{
			if (testFunc(_items[node->item], box))
				visitorFunc(_items[node->item]);

			// grab next node to process from todo stack
			if (todo_pos > 0)
			{
				node = todo[--todo_pos];
			} else
				break;
		} else
		{
			const Node *left = node + 1;
			const Node *right = &_nodes[node->child];

			// advance to next child node, possibly enqueue other child
			if (!left->bound.overlaps(box))
				node = right;
			else if (!right->bound.overlaps(box))
				node = left;
			else
			{
				// enqueue right child and continue with left
				todo[todo_pos++] = right;
				node = left;
			}
		}
	}
}

template<typename T>
void BVH<T>::forEachIntersectingItem(const mRay3 &ray, const Util::RayIntersectionTestFunc3<T> &testFunc, const Util::IntersectionVisitorFunc3<T> &visitorFunc) const
{
	if (!_bound.intersects(ray))
		return;

	// prepare to traverse BVH for ray
	static constexpr size_t MAX_TREE_DEPTH = 8 * sizeof(size_t);
	std::array<const Node *, MAX_TREE_DEPTH> todo;
	size_t todo_pos = 0;

	// traverse BVH nodes for ray
	const Node *node = _nodes.data();

	while (node != nullptr)
	{
		if (node->is_leaf())
		{
			if (testFunc(_items[node->item], ray))
				visitorFunc(_items[node->item]);

			// grab next node to process from todo stack
			if (todo_pos > 0)
			{
				node = todo[--todo_pos];
			} else
				break;
		} else
		{
			const Node *left;
			const Node *right;

			if (ray._direction[node->flags] > 0.0)
			{
				left = node + 1;
				right = (Node *) &_nodes[node->child];
			} else
			{
				left = (Node *) &_nodes[node->child];
				right = node + 1;
			}

			// advance to next child node, possibly enqueue other child
			if (!left->bound.intersects(ray))
				node = right;
			else if (!right->bound.intersects(ray))
				node = left;
			else
			{
				// enqueue secondChild in todo stack
				todo[todo_pos++] = right;
				node = left;
			}
		}
	}
}

template<typename T>
auto BVH<T>::closestIntersection(const mRay3 &ray, const Util::GetRayIntersectionFunc3<T> &testFunc) const -> Util::ClosestIntersectionQueryResult3<T>
{
	Util::ClosestIntersectionQueryResult3<T> best;

	if (!_bound.intersects(ray))
		return best;

	// prepare to traverse BVH for ray
	static constexpr size_t MAX_TREE_DEPTH = 8 * sizeof(size_t);
	std::array<const Node *, MAX_TREE_DEPTH> todo;
	size_t todo_pos = 0;

	// traverse BVH nodes for ray
	const Node *node = _nodes.data();

	while (node != nullptr)
	{
		if (node->is_leaf())
		{
			real dist = testFunc(_items[node->item], ray);
			if (dist < best.distance)
			{
				best.distance = dist;
				best.item = &_items[node->item];
			}

			// grab next node to process from todo stack
			if (todo_pos > 0)
			{
				node = todo[--todo_pos];
			} else
				break;
		} else {
			// get node children pointers for ray
			const Node *left;
			const Node *right;
			if (ray._direction[node->flags] > 0.0)
			{
				left = node + 1;
				right = (Node *) &_nodes[node->child];
			} else
			{
				left = (Node *) &_nodes[node->child];
				right = node + 1;
			}

			// advance to next child node, possibly enqueue other child
			if (!left->bound.intersects(ray))
				node = right;
			else if (!right->bound.intersects(ray))
				node = left;
			else
			{
				// enqueue secondChild in todo stack
				todo[todo_pos++] = right;
				node = left;
			}
		}
	}

	return best;
}

template<typename T>
Util::NearestNeighborQueryResult3<T> BVH<T>::nearest(const mVector3 &pt, const Util::NearestNeighborDistanceFunc3<T> &distanceFunc) const
{
	Util::NearestNeighborQueryResult3<T> best;

	// Prepare to traverse BVH
	static constexpr int MAX_TREE_DEPTH = 8 * sizeof(size_t);
	std::array<const Node *, MAX_TREE_DEPTH> todo;
	size_t todo_pos = 0;

	// Traverse BVH nodes
	const Node *node = _nodes.data();
	while (node != nullptr)
	{
		if (node->is_leaf())
		{
			real dist = distanceFunc(_items[node->item], pt);
			if (dist < best.distance)
			{
				best.distance = dist;
				best.item = &_items[node->item];
			}

			// Grab next node to process from todo stack
			if (todo_pos > 0)
			{
				// Dequeue
				--todo_pos;
				node = todo[todo_pos];
			} else
				break;
		} else
		{
			const real best_dist_sqr = best.distance * best.distance;

			const Node *left = node + 1;
			const Node *right = &_nodes[node->child];

			// If pt is inside the box, then the closestLeft and Right will be
			// identical to pt. This will make distMinLeftSqr and
			// distMinRightSqr zero, meaning that such a box will have higher
			// priority.
			mVector3 closest_left = left->bound.clamp(pt);
			mVector3 closest_right = right->bound.clamp(pt);

			real dist_min_left_sqr = (closest_left - pt).length_squared();
			real dist_min_right_sqr = (closest_right - pt).length_squared();

			bool should_visit_left = dist_min_left_sqr < best_dist_sqr;
			bool should_visit_right = dist_min_right_sqr < best_dist_sqr;

			const Node *first_child;
			const Node *second_child;
			if (should_visit_left && should_visit_right)
			{
				if (dist_min_left_sqr < dist_min_right_sqr)
				{
					first_child = left;
					second_child = right;
				} else
				{
					first_child = right;
					second_child = left;
				}

				// Enqueue second_child in todo stack
				todo[todo_pos] = second_child;
				++todo_pos;
				node = first_child;
			} else if (should_visit_left)
			{
				node = left;
			} else if (should_visit_right)
			{
				node = right;
			} else
			{
				if (todo_pos > 0)
				{
					// Dequeue
					--todo_pos;
					node = todo[todo_pos];
				} else
					break;
			}
		}
	}

	return best;
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
