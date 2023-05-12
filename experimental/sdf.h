#ifndef HINAPE_SDF_H
#define HINAPE_SDF_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "geom/grid.h"

namespace HinaPE::Experimental
{
class SDF final
{
public:
	// Generates a signed distance field from a mesh. Absolute distances will be nearly correct
	// for triangle soup, but a closed mesh is needed for accurate signs. Distances for all grid
	// cells within exact_band cells of a triangle should be exact, further away a distance is
	// calculated but it might not be to the closest triangle - just one nearby.
	static auto build(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, const Math::Size3& resolution, real spacing = 0.1, int exact_band = 1) -> Geom::DataGrid3<real>;
};
} // namespace HinaPE::Experimental

#endif //HINAPE_SDF_H
