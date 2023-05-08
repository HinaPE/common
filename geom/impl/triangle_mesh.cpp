#include "geom/triangle_mesh.h"
#include "igl/per_face_normals.h"

HinaPE::Geom::TriangleMeshSurface::TriangleMeshSurface(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices) : Surface3(), vertices(vertices), indices(indices)
{
	// generate normals
	Eigen::Matrix<real, Eigen::Dynamic, 3> V;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F;
	V.resize(vertices.size(), 3);
	F.resize(indices.size() / 3, 3);
	for (int i = 0; i < vertices.size(); i++)
	{
		V(i, 0) = vertices[i].x();
		V(i, 1) = vertices[i].y();
		V(i, 2) = vertices[i].z();
	}
	for (int i = 0; i < indices.size() / 3; i++)
	{
		F(i, 0) = indices[i * 3 + 0];
		F(i, 1) = indices[i * 3 + 1];
		F(i, 2) = indices[i * 3 + 2];
	}
	Eigen::Matrix<real, Eigen::Dynamic, 3> N;
	igl::per_face_normals(V, F, N);
	normals.resize(N.rows());
	for (int i = 0; i < N.rows(); i++)
		normals[i] = mVector3(N(i, 0), N(i, 1), N(i, 2));
}
auto HinaPE::Geom::TriangleMeshSurface::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	return mVector3();
}
auto HinaPE::Geom::TriangleMeshSurface::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return SurfaceRayIntersection3();
}
auto HinaPE::Geom::TriangleMeshSurface::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return mVector3();
}
auto HinaPE::Geom::TriangleMeshSurface::_intersects_local(const mRay3 &ray) const -> bool
{
	return false;
}
auto HinaPE::Geom::TriangleMeshSurface::_bounding_box_local() const -> mBBox3
{
	return mBBox3();
}

void HinaPE::Geom::TriangleMeshSurface::buildBVH() const
{
	size_t n_tris = indices.size() / 3;
	std::vector<size_t> ids(n_tris);
	std::vector<mBBox3> bounds(n_tris);
}
auto HinaPE::Geom::Triangle::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	return mVector3();
}
auto HinaPE::Geom::Triangle::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return SurfaceRayIntersection3();
}
auto HinaPE::Geom::Triangle::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return mVector3();
}
auto HinaPE::Geom::Triangle::_intersects_local(const mRay3 &ray) const -> bool
{
	return false;
}
auto HinaPE::Geom::Triangle::_bounding_box_local() const -> mBBox3
{
	return mBBox3();
}

auto HinaPE::Geom::Triangle::get_barycentric_coords(const mVector3 &pt) const -> std::tuple<real, real, real>
{
	auto q01 = (_points[1] - _points[0]).cross(pt - _points[0]);
	auto q12 = (_points[2] - _points[1]).cross(pt - _points[1]);
	auto q20 = (_points[0] - _points[2]).cross(pt - _points[2]);

	auto a = area();
	return {
			Constant::Half * q12.length() / a,
			Constant::Half * q20.length() / a,
			Constant::Half * q01.length() / a
	};
}

auto HinaPE::Geom::Triangle::area() const -> real
{
	auto &p1 = _points[0];
	auto &p2 = _points[1];
	auto &p3 = _points[2];

	return 0.5 * (p2 - p1).cross(p3 - p1).length();
}

auto HinaPE::Geom::Triangle::face_normal() const -> mVector3
{
	return ((_points[1] - _points[0]).cross(_points[2] - _points[0])).normalized();
}
