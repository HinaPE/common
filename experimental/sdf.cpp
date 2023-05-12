#include "sdf.h"

// find distance x0 is from segment x1-x2
static auto point_segment_distance(const mVector3 &x0, const mVector3 &x1, const mVector3 &x2) -> real
{
	mVector3 dx(x2 - x1);
	real m2 = dx.squared_norm();

	// find parameter value of closest point on segment
	real s12 = real((x2 - x0).dot(dx) / m2);
	if (s12 < 0)
		s12 = 0;
	else if (s12 > 1)
		s12 = 1;

	// and find the distance
	return (x0 - (s12 * x1 + (1 - s12) * x2)).norm();
}

// find distance x0 is from triangle x1-x2-x3
static auto point_triangle_distance(const mVector3 &x0, const mVector3 &x1, const mVector3 &x2, const mVector3 &x3) -> real
{
	// first find barycentric coordinates of closest point on infinite plane
	mVector3 x13(x1 - x3), x23(x2 - x3), x03(x0 - x3);
	real m13 = x13.squared_norm(), m23 = x23.squared_norm(), d = x13.dot(x23);
	real invdet = 1.f / std::max(m13 * m23 - d * d, HinaPE::Constant::Epsilon);
	real a = x13.dot(x03), b = x23.dot(x03);
	// the barycentric coordinates themselves
	real w23 = invdet * (m23 * a - d * b);
	real w31 = invdet * (m13 * b - d * a);
	real w12 = 1 - w23 - w31;
	if (w23 >= 0 && w31 >= 0 && w12 >= 0)
	{ 	// if we're inside the triangle
		return (x0 - (w23 * x1 + w31 * x2 + w12 * x3)).norm();
	} else
	{ // we have to clamp to one of the edges
		if (w23 > 0) // this rules out edge 2-3 for us
			return std::min(point_segment_distance(x0, x1, x2), point_segment_distance(x0, x1, x3));
		else if (w31 > 0) // this rules out edge 1-3
			return std::min(point_segment_distance(x0, x1, x2), point_segment_distance(x0, x2, x3));
		else // w12 must be >0, ruling out edge 1-2
			return std::min(point_segment_distance(x0, x1, x3), point_segment_distance(x0, x2, x3));
	}
}

static void check_neighbour(const std::vector<mVector3> &vertices, const std::vector<mVector3> &indices,
							HinaPE::Geom::DataGrid3<real> &sdf, HinaPE::Geom::DataGrid3<real> &closest_tri,
							const mVector3 &gx, int i0, int j0, int k0, int i1, int j1, int k1) {
//	if (closest_tri(i1, j1, k1) >= 0) {
//		const Vector3u &triangle = mesh.triangles().col(closest_tri(i1, j1, k1));
//		float d = point_triangle_distance(gx, mesh.vertices().col(triangle[0]), mesh.vertices().col(triangle[1]), mesh.vertices().col(triangle[2]));
//		if (d < sdf(i0, j0, k0)) {
//			sdf(i0, j0, k0) = d;
//			closest_tri(i0, j0, k0) = closest_tri(i1, j1, k1);
//		}
//	}
}

auto HinaPE::Experimental::SDF::build(const std::vector<mVector3> &vertices, const std::vector<mVector3> &indices, const Math::Size3 &resolution, real spacing, const int exact_band) -> Geom::DataGrid3 <real>
{
	Geom::DataGrid3<real> sdf;
	sdf.resize(resolution, spacing * mVector3::One());

	auto ni = resolution.x;
	auto nj = resolution.y;
	auto nk = resolution.z;
	auto origin = sdf.origin;
	auto dx = sdf.spacing;

	sdf.clear(std::numeric_limits<real>::max());


	return sdf;
}
