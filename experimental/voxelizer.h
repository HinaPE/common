#ifndef HINAPE_VOXELIZER_H
#define HINAPE_VOXELIZER_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "common.h"

namespace HinaPE::Experimental
{
class Voxelizer final
{
public:
	struct Result
	{
		mVector3 spacing;
		mBBox3 bounds;
		Geom::DataGrid3<int> grid; //  1 for true, 0 for false
	};

	static void voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing, Result &result);
	static void voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing, std::vector<mVector3> &voxels);
};
} // namespace HinaPE::Experimental

#endif //HINAPE_VOXELIZER_H
