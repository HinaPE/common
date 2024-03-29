#include "geom/triangle_mesh.h"
#include "igl/per_face_normals.h"
#include "igl/per_vertex_normals.h"

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

HinaPE::Geom::TriangleMeshSurface::TriangleMeshSurface(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices) : Surface3(), _vertices(vertices), _indices(indices)
{
	reload(vertices, indices);
}

void HinaPE::Geom::TriangleMeshSurface::reload(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices)
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
	{ // per face normals
		Eigen::Matrix<real, Eigen::Dynamic, 3> N;
		igl::per_face_normals(V, F, N);
		_normals.resize(N.rows());
		for (int i = 0; i < N.rows(); i++)
			_normals[i] = mVector3(N(i, 0), N(i, 1), N(i, 2));
	}
	{ // per vertex normals
		Eigen::Matrix<real, Eigen::Dynamic, 3> N;
		igl::per_vertex_normals(V, F, N);
		_normals_per_vertex.resize(N.rows());
		for (int i = 0; i < N.rows(); i++)
			_normals_per_vertex[i] = mVector3(N(i, 0), N(i, 1), N(i, 2));
	}
}

auto HinaPE::Geom::TriangleMeshSurface::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	buildBVH();

	const auto distance_func = [this](const size_t &triIdx, const mVector3 &pt)
	{
		Triangle tri = triangle(triIdx);
		return tri.closest_distance(pt);
	};

	const auto query_res = _bvh.nearest(other_point, distance_func);

	return triangle(*query_res.item).closest_point(other_point);
}

auto HinaPE::Geom::TriangleMeshSurface::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	buildBVH();

	const auto test_func = [this](const size_t &triIdx, const mRay3 &ray)
	{
		Triangle tri = triangle(triIdx);
		return tri.closest_intersection(ray).distance;
	};

	const auto query_res = _bvh.closestIntersection(ray, test_func);
	SurfaceRayIntersection3 res;
	res.distance = query_res.distance;
	res.is_intersecting = query_res.item != nullptr;
	if (res.is_intersecting)
	{
		res.point = ray.point_at(query_res.distance);
		res.normal = triangle(*query_res.item).closest_normal(res.point);
	}
	return res;
}

auto HinaPE::Geom::TriangleMeshSurface::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	buildBVH();

	const auto distance_func = [this](const size_t &triIdx, const mVector3 &pt)
	{
		Triangle tri = triangle(triIdx);
		return tri.closest_distance(pt);
	};

	const auto query_res = _bvh.nearest(other_point, distance_func);

	return triangle(*query_res.item).closest_normal(other_point);
}

auto HinaPE::Geom::TriangleMeshSurface::_intersects_local(const mRay3 &ray) const -> bool
{
	buildBVH();

	const auto test_func = [this](const size_t &triIdx, const mRay3 &ray)
	{
		Triangle tri = triangle(triIdx);
		return tri.intersects(ray);
	};

	return _bvh.intersects(ray, test_func);
}

auto HinaPE::Geom::TriangleMeshSurface::_bounding_box_local() const -> mBBox3
{
	buildBVH();

	return _bvh.bbox();
}

void HinaPE::Geom::TriangleMeshSurface::buildBVH() const
{
	if (_bvh_dirty)
	{
		size_t n_tris = _indices.size() / 3;
		std::vector<size_t> ids(n_tris);
		std::vector<mBBox3> bounds(n_tris);
		for (int i = 0; i < n_tris; ++i)
		{
			ids[i] = i;
			bounds[i] = triangle(i).bounding_box();
		}
		_bvh.build(ids, bounds);
		_bvh_dirty = false;
	}
}

auto HinaPE::Geom::TriangleMeshSurface::triangle(size_t i) const -> HinaPE::Geom::Triangle
{
	Triangle triangle;

	// TODO: not completed
	for (int j = 0; j < 3; ++j)
	{
		triangle._points[i] = _vertices[_indices[i * 3 + j]];
		triangle._normals[i] = _normals[_indices[i * 3 + j]];
		triangle._uvs[i] = _uvs[_indices[i * 3 + j]];
	}

	return triangle;
}

auto HinaPE::Geom::ImplicitTriangleMeshSurface::_closest_point_local(const mVector3 &other_point) const -> mVector3
{
	return mVector3();
}

auto HinaPE::Geom::ImplicitTriangleMeshSurface::_closest_intersection_local(const mRay3 &ray) const -> HinaPE::Geom::SurfaceRayIntersection3
{
	return SurfaceRayIntersection3();
}

auto HinaPE::Geom::ImplicitTriangleMeshSurface::_closest_normal_local(const mVector3 &other_point) const -> mVector3
{
	return mVector3();
}

auto HinaPE::Geom::ImplicitTriangleMeshSurface::_intersects_local(const mRay3 &ray) const -> bool
{
	return false;
}

auto HinaPE::Geom::ImplicitTriangleMeshSurface::_bounding_box_local() const -> mBBox3
{
	return mBBox3();
}

auto HinaPE::Geom::ImplicitTriangleMeshSurface::_signed_distance_local(const mVector3 &other_point) const -> real
{
	return 0;
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
