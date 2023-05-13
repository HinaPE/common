#include "voxelizer.h"

#include "experimental/Mesh.h"
#include "experimental/Voxelizer.h"

auto HinaPE::Util::Voxelizer::voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing) -> Geom::DataGrid3<int>
{
	Geom::DataGrid3<int> data;

	cs224::Mesh mesh;
	for (auto &v : vertices)
		mesh.addVertex(cs224::Vector3f(v.x(), v.y(), v.z()), cs224::Vector3f(), cs224::Vector2f());

	for (int i = 0; i < indices.size(); i+=3)
		mesh.addTriangle(indices[i], indices[i+1], indices[i+2]);

	cs224::Voxelizer::Result result;
	cs224::Voxelizer::voxelize(mesh, spacing.x(), result);

	auto grid = result.grid;
	auto bounds = result.bounds;
	auto cellSize = result.cellSize;
	auto sp = {cellSize, cellSize, cellSize};
	HinaPE::Math::Size3 resolution = {grid.size().x(), grid.size().y(), grid.size().z()};
	data.resize(resolution, sp);

	for (int z = 0; z < resolution.z; ++z)
		for (int y = 0; y < resolution.y; ++y)
			for (int x = 0; x < resolution.x; ++x)
				data(x, y, z) = grid.value(x, y, z);

	return data;
}

void HinaPE::Util::Voxelizer::voxelize(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, mVector3 spacing, std::vector<mVector3> &voxels)
{

}
