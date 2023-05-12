#include "voxelizer.h"

void HinaPE::Experimental::Voxelizer::voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing, HinaPE::Experimental::Voxelizer::Result &result)
{
	result.spacing = spacing;
	result.bounds = mBBox3(vertices);

	Math::Size3 resolution;
	resolution.x = static_cast<size_t>(std::ceil(result.bounds.width() / spacing.x()));
	resolution.y = static_cast<size_t>(std::ceil(result.bounds.height() / spacing.y()));
	resolution.z = static_cast<size_t>(std::ceil(result.bounds.depth() / spacing.z()));

	result.grid.resize(resolution, spacing, result.bounds.center());
	result.grid.clear(false);

	// 3d index of the voxel containing the point
	auto index = [&](const mVector3 &p) -> mVector3i
	{
		return {
				static_cast<int>(std::floor((p.x() - result.bounds._lower_corner.x()) / result.spacing.x())),
				static_cast<int>(std::floor((p.y() - result.bounds._lower_corner.y()) / result.spacing.y())),
				static_cast<int>(std::floor((p.z() - result.bounds._lower_corner.z()) / result.spacing.z()))
		};
	};

	auto rasterize_triangle = [&](const mVector3 &p0, const mVector3 &p1, const mVector3 &p2)
	{
		mBBox3i iBounds;
		iBounds.merge(index(p0));
		iBounds.merge(index(p1));
		iBounds.merge(index(p2));
		mVector3 e0 = p1 - p0;
		mVector3 e1 = p2 - p0;
		mVector3 normal(e0.cross(e1).normalized());

		for (int z = iBounds._lower_corner.z(); z <= iBounds._upper_corner.z(); ++z)
		{
			for (int y = iBounds._lower_corner.y(); y <= iBounds._upper_corner.y(); ++y)
			{
				for (int x = iBounds._lower_corner.x(); x <= iBounds._upper_corner.x(); ++x)
				{
					mVector3 center = result.bounds.center();
					real d = (center - p0).dot(normal);

					// check if voxel intersects triangle plane
					if (std::abs(d) < 0.70710678118655f * spacing.x() * 2) // TODO: error prone
					{
						// check if projected voxel center lies within triangle
						mVector3 p = center - normal * d;
						real inv_two_area = Constant::One / (e0.cross(e1)).norm();
						real s = inv_two_area * (p0.y() * p2.x() - p0.x() * p2.y() + (p2.y() - p0.y()) * p.x() + (p0.x() - p2.x()) * p.y());
						real t = inv_two_area * (p0.x() * p1.y() - p0.y() * p1.x() + (p0.y() - p1.y()) * p.x() + (p1.x() - p0.x()) * p.y());

						// account for counter-clockwise triangles
						if (s < 0.f && t < 0.f)
						{
							s = -s;
							t = -t;
						}

						if (s >= 0.f && t >= 0.f && (1.f - s - t) >= 0.f)
							result.grid(x, y, z) = true;
					}
				}
			}
		}
	};

	for (int t = 0; t < indices.size(); ++t)
	{
		const mVector3 &p0 = vertices[indices[t + 1]];
		const mVector3 &p1 = vertices[indices[t + 2]];
		const mVector3 &p2 = vertices[indices[t + 3]];
		rasterize_triangle(p0, p1, p2);
	}
}

void HinaPE::Experimental::Voxelizer::voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing, std::vector<mVector3> &voxels)
{
	Result result;
	voxelize(vertices, indices, spacing, result);
	auto resolution = result.grid.resolution;

	for (int z = 0; z < resolution.z; ++z)
		for (int y = 0; y < resolution.y; ++y)
			for (int x = 0; x < resolution.x; ++x)
				if (result.grid(x, y, z)) // 1 for true, 0 for false
					voxels.emplace_back(result.bounds._lower_corner + mVector3(x + Constant::Half, y + Constant::Half, z + Constant::Half) * spacing);
}
