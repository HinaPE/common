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
	static auto voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing) -> Geom::DataGrid3<int>; //  1 for true, 0 for false
	static void voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing, std::vector<mVector3> &voxels);
};
} // namespace HinaPE::Experimental

#endif //HINAPE_VOXELIZER_H
