#ifndef HINAPE_BVH_H
#define HINAPE_BVH_H

#include "bbox.h"

namespace HinaPE::Geom
{
class BVH
{
public:
	struct Node
	{
		char flags;
		union
		{
			size_t child;
			size_t item;
		};
		mBBox3 bound;

		Node();
		void init_leaf(size_t it, const mBBox3& b);
		void init_internal(uint8_t axis, size_t c, const mBBox3& b);
		bool is_leaf() const;
	};
};
} // namespace HinaPE::Geom

#endif //HINAPE_BVH_H
