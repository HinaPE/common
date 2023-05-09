#include "geom/triangle_mesh.h"
#include "igl/per_face_normals.h"

inline auto closest_point_on_line(const mVector3 &v0, const mVector3 &v1, const mVector3 &pt) -> mVector3
{
	const real len_square = (v1 - v0).length_squared();
	if (len_square < HinaPE::Constant::Epsilon)
		return v0;

	const real t = (pt - v0).dot(v1 - v0) / len_square;
	if (t < 0.0)
		return v0;
	else if (t > 1.0)
		return v1;
	else
		return v0 + t * (v1 - v0);
}

inline auto closest_normal_on_line(const mVector3 &v0, const mVector3 &v1, const mVector3 &n0, const mVector3 &n1, const mVector3 &pt) -> mVector3
{
	const real len_square = (v1 - v0).length_squared();
	if (len_square < HinaPE::Constant::Epsilon)
		return n0;

	const real t = (pt - v0).dot(v1 - v0) / len_square;
	if (t < 0.0)
		return n0;
	else if (t > 1.0)
		return n1;
	else
		return (n0 + t * (n1 - n0)).normalized();
}

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
	auto n = face_normal();
	real nd = n.dot(n);
	real d = n.dot(_points[0]);
	real t = (d - n.dot(other_point)) / nd;

	auto q = other_point + t * n;

	auto q01 = (_points[1] - _points[0]).cross(q - _points[0]);
	auto q12 = (_points[2] - _points[1]).cross(q - _points[1]);
	auto q20 = (_points[0] - _points[2]).cross(q - _points[2]);

	if (n.dot(q01) < 0) return closest_point_on_line(_points[0], _points[1], q);
	if (n.dot(q12) < 0) return closest_point_on_line(_points[1], _points[2], q);
	if (n.dot(q20) < 0) return closest_point_on_line(_points[2], _points[0], q);

	real a = area();
	real b0 = Constant::Half * q12.length() / a;
	real b1 = Constant::Half * q20.length() / a;
	real b2 = Constant::Half * q01.length() / a;

	return b0 * _points[0] + b1 * _points[1] + b2 * _points[2];
}
auto HinaPE::Geom::Triangle::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	SurfaceRayIntersection3 intersection;
	auto n = face_normal();
	real nd = n.dot(ray._direction);

	if (nd < Constant::Epsilon)
		return intersection; // no intersection

	real d = n.dot(_points[0]);
	real t = (d - n.dot(ray._origin)) / nd;

	if (t < 0)
		return intersection; // no intersection

	auto q = ray.point_at(t);

	auto q01 = (_points[1] - _points[0]).cross(q - _points[0]);
	auto q12 = (_points[2] - _points[1]).cross(q - _points[1]);
	auto q20 = (_points[0] - _points[2]).cross(q - _points[2]);

	if (n.dot(q01) < 0) return intersection;
	if (n.dot(q12) < 0) return intersection;
	if (n.dot(q20) < 0) return intersection;

	real a = area();
	real b0 = Constant::Half * q12.length() / a;
	real b1 = Constant::Half * q20.length() / a;
	real b2 = Constant::Half * q01.length() / a;

	auto normal = b0 * _normals[0] + b1 * _normals[1] + b2 * _normals[2];

	intersection.is_intersecting = true;
	intersection.distance = t;
	intersection.point = q;
	intersection.normal = normal.normalized();

	return intersection;
}
auto HinaPE::Geom::Triangle::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	auto n = face_normal();
	real nd = n.dot(n);
	real d = n.dot(_points[0]);
	real t = (d - n.dot(other_point)) / nd;

	auto q = other_point + t * n;

	auto q01 = (_points[1] - _points[0]).cross(q - _points[0]);
	auto q12 = (_points[2] - _points[1]).cross(q - _points[1]);
	auto q20 = (_points[0] - _points[2]).cross(q - _points[2]);

	if (n.dot(q01) < 0) return closest_normal_on_line(_points[0], _points[1], _normals[0], _normals[1], q);
	if (n.dot(q12) < 0) return closest_normal_on_line(_points[1], _points[2], _normals[1], _normals[2], q);
	if (n.dot(q20) < 0) return closest_normal_on_line(_points[2], _points[0], _normals[2], _normals[0], q);

	real a = area();
	real b0 = Constant::Half * q12.length() / a;
	real b1 = Constant::Half * q20.length() / a;
	real b2 = Constant::Half * q01.length() / a;

	return (b0 * _normals[0] + b1 * _normals[1] + b2 * _normals[2]).normalized();
}
auto HinaPE::Geom::Triangle::_intersects_local(const mRay3 &ray) const -> bool
{
	auto n = face_normal();
	real nd = n.dot(ray._direction);
	if (nd < Constant::Epsilon)
		return false;

	real d = n.dot(_points[0]);
	real t = (d - n.dot(ray._origin)) / nd;
	if (t < 0)
		return false;

	auto q = ray.point_at(t);

	auto q01 = (_points[1] - _points[0]).cross(q - _points[0]);
	auto q12 = (_points[2] - _points[1]).cross(q - _points[1]);
	auto q20 = (_points[0] - _points[2]).cross(q - _points[2]);

	if (n.dot(q01) <= 0) return false;
	if (n.dot(q12) <= 0) return false;
	if (n.dot(q20) <= 0) return false;

	return true;
}
auto HinaPE::Geom::Triangle::_bounding_box_local() const -> mBBox3
{
	mBBox3 res(_points[0], _points[1]);
	res.merge(_points[2]);
	return res;
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
