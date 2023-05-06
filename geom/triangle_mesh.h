#ifndef HINAPE_TRIANGLE_MESH_H
#define HINAPE_TRIANGLE_MESH_H

#include "surface3.h"
#include "bvh.h"

namespace HinaPE::Geom
{
class TriangleMeshSurface : public Surface3
{
public:
	TriangleMeshSurface(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices);

protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;

private:
	std::vector<mVector3> vertices;
	std::vector<mVector3> normals;
	std::vector<mVector2> uvs;
	std::vector<unsigned int> indices;
	std::vector<unsigned int> normal_indices;
	std::vector<unsigned int> uv_indices;
};
} // namespace HinaPE::Geom

#endif //HINAPE_TRIANGLE_MESH_H
