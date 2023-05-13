#ifndef HINAPE_TRIANGLE_MESH_H
#define HINAPE_TRIANGLE_MESH_H

#include "surface3.h"
#include "grid.h"
#include "bvh.h"

namespace HinaPE::Geom
{
class Triangle;
class TriangleMeshSurface : public Surface3
{
public:
	TriangleMeshSurface() = default;
	TriangleMeshSurface(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices);
	void reload(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices);
	auto triangle(size_t i) const -> Triangle;
	auto normals_per_face() const -> const std::vector<mVector3> &{ return _normals; }
	auto normals_per_vertex() const -> const std::vector<mVector3> &{ return _normals_per_vertex; }
	inline auto number_of_triangles() -> size_t{return _indices.size() / 3;}

protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;

private:
	void buildBVH() const;

private:
	std::vector<mVector3> _vertices;
	std::vector<mVector3> _normals;
	std::vector<mVector3> _normals_per_vertex;
	std::vector<mVector2> _uvs;
	std::vector<unsigned int> _indices;
	std::vector<unsigned int> _normal_indices;
	std::vector<unsigned int> _uv_indices;

	mutable BVH<size_t> _bvh;
	mutable bool _bvh_dirty = true;
};

class ImplicitTriangleMeshSurface : public ImplicitSurface3
{
protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 override;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 override;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 override;
	auto _intersects_local(const mRay3 &ray) const -> bool override;
	auto _bounding_box_local() const -> mBBox3 override;
	auto _signed_distance_local(const mVector3 &other_point) const -> real override;

private:
	std::shared_ptr<TriangleMeshSurface> _mesh;
	std::vector<VectorGridField3> _grid;
};

class Triangle final : public Surface3
{
public:
	auto get_barycentric_coords(const mVector3 &pt) const -> std::tuple<real, real, real>;
	auto area() const -> real;
	auto face_normal() const -> mVector3;

protected:
	auto _closest_point_local(const mVector3 &other_point) const -> mVector3 final;
	auto _closest_intersection_local(const mRay3 &ray) const -> SurfaceRayIntersection3 final;
	auto _closest_normal_local(const mVector3 &other_point) const -> mVector3 final;
	auto _intersects_local(const mRay3 &ray) const -> bool final;
	auto _bounding_box_local() const -> mBBox3 final;

public:
	std::array<mVector3, 3> _points;
	std::array<mVector3, 3> _normals;
	std::array<mVector2, 3> _uvs;
};
} // namespace HinaPE::Geom

#endif //HINAPE_TRIANGLE_MESH_H
