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
mVector3 HinaPE::Geom::TriangleMeshSurface::_closest_point_local(const mVector3 &other_point) const
{
	return mVector3();
}
HinaPE::Geom::SurfaceRayIntersection3 HinaPE::Geom::TriangleMeshSurface::_closest_intersection_local(const mRay3 &ray) const
{
	return SurfaceRayIntersection3();
}
mVector3 HinaPE::Geom::TriangleMeshSurface::_closest_normal_local(const mVector3 &other_point) const
{
	return mVector3();
}
bool HinaPE::Geom::TriangleMeshSurface::_intersects_local(const mRay3 &ray) const
{
	return false;
}
mBBox3 HinaPE::Geom::TriangleMeshSurface::_bounding_box_local() const
{
	return mBBox3();
}
