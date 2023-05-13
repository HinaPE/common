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
	{    // if we're inside the triangle
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

static void check_neighbour(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices,
							HinaPE::Geom::DataGrid3<real> &sdf, HinaPE::Geom::DataGrid3<int> &closest_tri,
							const mVector3 &gx, int i0, int j0, int k0, int i1, int j1, int k1)
{
	if (closest_tri(i1, j1, k1) >= 0)
	{
		auto t1 = indices[closest_tri(i1, j1, k1) + 0];
		auto t2 = indices[closest_tri(i1, j1, k1) + 1];
		auto t3 = indices[closest_tri(i1, j1, k1) + 2];
		real d = point_triangle_distance(gx, vertices[t1], vertices[t2], vertices[t3]);
		if (d < sdf(i0, j0, k0))
		{
			sdf(i0, j0, k0) = d;
			closest_tri(i0, j0, k0) = closest_tri(i1, j1, k1);
		}
	}
}

static void sweep(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, HinaPE::Geom::DataGrid3<real> &sdf, HinaPE::Geom::DataGrid3<int> &closest_tri, const mVector3 &origin, mVector3 dx, int di, int dj, int dk)
{
	int i0, i1, j0, j1, k0, k1;
	if (di > 0)
	{
		i0 = 1;
		i1 = sdf.resolution.x;
	} else
	{
		i0 = sdf.resolution.x - 2;
		i1 = -1;
	}
	if (dj > 0)
	{
		j0 = 1;
		j1 = sdf.resolution.y;
	} else
	{
		j0 = sdf.resolution.y - 2;
		j1 = -1;
	}
	if (dk > 0)
	{
		k0 = 1;
		k1 = sdf.resolution.z;
	} else
	{
		k0 = sdf.resolution.z - 2;
		k1 = -1;
	}
	for (int k = k0; k != k1; k += dk)
	{
		for (int j = j0; j != j1; j += dj)
		{
			for (int i = i0; i != i1; i += di)
			{
				mVector3 gx = origin + mVector3(i * dx.x(), j * dx.y(), k * dx.z());
				check_neighbour(vertices, indices, sdf, closest_tri, gx, i, j, k, i - di, j, k);
				check_neighbour(vertices, indices, sdf, closest_tri, gx, i, j, k, i, j - dj, k);
				check_neighbour(vertices, indices, sdf, closest_tri, gx, i, j, k, i - di, j - dj, k);
				check_neighbour(vertices, indices, sdf, closest_tri, gx, i, j, k, i, j, k - dk);
				check_neighbour(vertices, indices, sdf, closest_tri, gx, i, j, k, i - di, j, k - dk);
				check_neighbour(vertices, indices, sdf, closest_tri, gx, i, j, k, i, j - dj, k - dk);
				check_neighbour(vertices, indices, sdf, closest_tri, gx, i, j, k, i - di, j - dj, k - dk);
			}
		}
	}
}

// calculate twice signed area of triangle (0,0)-(x1,y1)-(x2,y2)
// return an SOS-determined sign (-1, +1, or 0 only if it's a truly degenerate triangle)
static auto orientation(real x1, real y1, real x2, real y2, real &twice_signed_area) -> int
{
	twice_signed_area = y1 * x2 - x1 * y2;
	if (twice_signed_area > 0) return 1;
	else if (twice_signed_area < 0) return -1;
	else if (y2 > y1) return 1;
	else if (y2 < y1) return -1;
	else if (x1 > x2) return 1;
	else if (x1 < x2) return -1;
	else return 0; // only true when x1==x2 and y1==y2
}

// robust test of (x0,y0) in the triangle (x1,y1)-(x2,y2)-(x3,y3)
// if true is returned, the barycentric coordinates are set in a,b,c.
static auto point_in_triangle_2d(real x0, real y0, real x1, real y1, real x2, real y2, real x3, real y3, real &a, real &b, real &c) -> bool
{
	x1 -= x0;
	x2 -= x0;
	x3 -= x0;
	y1 -= y0;
	y2 -= y0;
	y3 -= y0;
	int signa = orientation(x2, y2, x3, y3, a);
	if (signa == 0) return false;
	int signb = orientation(x3, y3, x1, y1, b);
	if (signb != signa) return false;
	int signc = orientation(x1, y1, x2, y2, c);
	if (signc != signa) return false;
	real sum = a + b + c;
	assert(sum != 0); // if the SOS signs match and are nonkero, there's no way all of a, b, and c are zero.
	a /= sum;
	b /= sum;
	c /= sum;
	return true;
}

auto HinaPE::Util::Legacy::SDF::build(const std::vector<mVector3> &vertices, const std::vector<unsigned int> &indices, const Math::Size3 &resolution, real spacing, const int exact_band) -> Geom::DataGrid3 <real>
{
	// init sdf
	Geom::DataGrid3<real> sdf;
	sdf.resize(resolution, spacing * mVector3::One());
	sdf.clear(std::numeric_limits<real>::max());

	// init the closest triangle
	Geom::DataGrid3<int> closest_tri;
	closest_tri.resize(resolution, spacing * mVector3::One());
	closest_tri.clear(-1);

	Geom::DataGrid3<int> intersection_count;
	closest_tri.resize(resolution, spacing * mVector3::One());
	closest_tri.clear(0);

	auto ni = resolution.x;
	auto nj = resolution.y;
	auto nk = resolution.z;
	auto origin = sdf.origin;
	auto dx = sdf.spacing;

	mVector3 ijkmin, ijkmax;
	for (int t = 0; t < indices.size(); t += 3)
	{
		const auto &t1 = indices[t + 0];
		const auto &t2 = indices[t + 1];
		const auto &t3 = indices[t + 2];

		const auto &v1 = vertices[t1];
		const auto &v2 = vertices[t2];
		const auto &v3 = vertices[t3];

		real fip = (v1.x() - origin.x()) / dx.x(), fjp = (v1.y() - origin.y()) / dx.y(), fkp = (v1.z() - origin.z()) / dx.z();
		real fiq = (v2.x() - origin.x()) / dx.x(), fjq = (v2.y() - origin.y()) / dx.y(), fkq = (v2.z() - origin.z()) / dx.z();
		real fir = (v3.x() - origin.x()) / dx.x(), fjr = (v3.y() - origin.y()) / dx.y(), fkr = (v3.z() - origin.z()) / dx.z();

		// do distances nearby
		int i0 = Math::clamp(static_cast<size_t>(std::min(fip, std::min(fiq, fir))) - exact_band, (size_t) 0, ni - 1), i1 = Math::clamp(static_cast<size_t>(std::max(fip, std::max(fiq, fir))) + exact_band, (size_t) 0, ni - 1);
		int j0 = Math::clamp(static_cast<size_t>(std::min(fjp, std::min(fjq, fjr))) - exact_band, (size_t) 0, nj - 1), j1 = Math::clamp(static_cast<size_t>(std::max(fjp, std::max(fjq, fjr))) + exact_band, (size_t) 0, nj - 1);
		int k0 = Math::clamp(static_cast<size_t>(std::min(fkp, std::min(fkq, fkr))) - exact_band, (size_t) 0, nk - 1), k1 = Math::clamp(static_cast<size_t>(std::max(fkp, std::max(fkq, fkr))) + exact_band, (size_t) 0, nk - 1);

		for (int k = k0; k <= k1; ++k)
		{
			for (int j = j0; j <= j1; ++j)
			{
				for (int i = i0; i <= i1; ++i)
				{
					mVector3 gx(i * dx.x() + origin.x(), j * dx.y() + origin.y(), k * dx.z() + origin.z());
					real d = point_triangle_distance(gx, v1, v2, v3);
					if (d < sdf(i, j, k))
					{
						sdf(i, j, k) = d;
						closest_tri(i, j, k) = t;
					}
				}
			}
		}

		// and do intersection counts
		j0 = Math::clamp(static_cast<size_t>(std::ceil(std::min(fjp, std::min(fjq, fjr)))), (size_t) 0, nj - 1);
		j1 = Math::clamp(static_cast<size_t>(std::floor(std::max(fjp, std::max(fjq, fjr)))), (size_t) 0, nj - 1);
		k0 = Math::clamp(static_cast<size_t>(std::ceil(std::min(fkp, std::min(fkq, fkr)))), (size_t) 0, nk - 1);
		k1 = Math::clamp(static_cast<size_t>(std::floor(std::max(fkp, std::max(fkq, fkr)))), (size_t) 0, nk - 1);
		for (int k = k0; k <= k1; ++k)
		{
			for (int j = j0; j <= j1; ++j)
			{
				real a, b, c;
				if (point_in_triangle_2d(j, k, fjp, fkp, fjq, fkq, fjr, fkr, a, b, c))
				{
					real fi = a * fip + b * fiq + c * fir; // intersection point
					int i_interval = static_cast<int>(std::ceil(fi)); // intersection is in (i_interval-1,i_interval]
					if (i_interval < 0)
						++intersection_count(0, j, k); // we enlarge the first interval to include everything to the -x direction
					else if (i_interval < ni)
						++intersection_count(i_interval, j, k);
					// we ignore intersections that are beyond the +x side of the grid
				}
			}
		}

		// and now we fill in the rest of the distances with fast sweeping
		for (unsigned int pass = 0; pass < 2; ++pass)
		{
			sweep(vertices, indices, sdf, closest_tri, origin, dx, +1, +1, +1);
			sweep(vertices, indices, sdf, closest_tri, origin, dx, -1, -1, -1);
			sweep(vertices, indices, sdf, closest_tri, origin, dx, +1, +1, -1);
			sweep(vertices, indices, sdf, closest_tri, origin, dx, -1, -1, +1);
			sweep(vertices, indices, sdf, closest_tri, origin, dx, +1, -1, +1);
			sweep(vertices, indices, sdf, closest_tri, origin, dx, -1, +1, -1);
			sweep(vertices, indices, sdf, closest_tri, origin, dx, +1, -1, -1);
			sweep(vertices, indices, sdf, closest_tri, origin, dx, -1, +1, +1);
		}

		// then figure out signs (inside/outside) from intersection counts
		for (int k = 0; k < nk; ++k)
		{
			for (int j = 0; j < nj; ++j)
			{
				auto total_count = 0;
				for (int i = 0; i < ni; ++i)
				{
					total_count += intersection_count(i, j, k);
					if (total_count % 2 == 1)
					{
						// if parity of intersections so far is odd,
						// we are inside the mesh
						sdf(i, j, k) = -sdf(i, j, k);
					}
				}
			}
		}
	}

	return sdf;
}
