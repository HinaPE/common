// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2016 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_RAY_MESH_INTERSECT_H
#define IGL_RAY_MESH_INTERSECT_H
#include "igl_inline.h"
#include "Hit.h"
#include <Eigen/Core>
#include <vector>

/* Ray-Triangle Intersection Test Routines          */
/* Different optimizations of my and Ben Trumbore's */
/* code from journals of graphics tools (JGT)       */
/* http://www.acm.org/jgt/                          */
/* by Tomas Moller, May 2000                        */


// Alec: this file is listed as "Public Domain"
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/

// Alec: I've added an include guard, made all functions inline and added
// IGL_RAY_TRI_ to #define macros
#ifndef IGL_RAY_TRI_C
#define IGL_RAY_TRI_C

#include <cmath>

#define IGL_RAY_TRI_EPSILON 0.000001
#define IGL_RAY_TRI_CROSS(dest, v1, v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define IGL_RAY_TRI_DOT(v1, v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define IGL_RAY_TRI_SUB(dest, v1, v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2];

/* the original jgt code */
inline int intersect_triangle(double orig[3], double dir[3],
							  double vert0[3], double vert1[3], double vert2[3],
							  double *t, double *u, double *v)
{
	double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	double det, inv_det;

	/* find vectors for two edges sharing vert0 */
	IGL_RAY_TRI_SUB(edge1, vert1, vert0);
	IGL_RAY_TRI_SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	IGL_RAY_TRI_CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = IGL_RAY_TRI_DOT(edge1, pvec);

	if (det > -IGL_RAY_TRI_EPSILON && det < IGL_RAY_TRI_EPSILON)
		return 0;
	inv_det = 1.0 / det;

	/* calculate distance from vert0 to ray origin */
	IGL_RAY_TRI_SUB(tvec, orig, vert0);

	/* calculate U parameter and test bounds */
	*u = IGL_RAY_TRI_DOT(tvec, pvec) * inv_det;
	if (*u < 0.0 || *u > 1.0)
		return 0;

	/* prepare to test V parameter */
	IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

	/* calculate V parameter and test bounds */
	*v = IGL_RAY_TRI_DOT(dir, qvec) * inv_det;
	if (*v < 0.0 || *u + *v > 1.0)
		return 0;

	/* calculate t, ray intersects triangle */
	*t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;

	return 1;
}


/* code rewritten to do tests on the sign of the determinant */
/* the division is at the end in the code                    */
inline int intersect_triangle1(double orig[3], double dir[3],
							   double vert0[3], double vert1[3], double vert2[3],
							   double *t, double *u, double *v)
{
	double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	double det, inv_det;

	/* find vectors for two edges sharing vert0 */
	IGL_RAY_TRI_SUB(edge1, vert1, vert0);
	IGL_RAY_TRI_SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	IGL_RAY_TRI_CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = IGL_RAY_TRI_DOT(edge1, pvec);

	if (det > IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (*u < 0.0 || *u > det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v < 0.0 || *u + *v > det)
			return 0;
	} else if (det < -IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
/*      printf("*u=%f\n",(float)*u); */
/*      printf("det=%f\n",det); */
		if (*u > 0.0 || *u < det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v > 0.0 || *u + *v < det)
			return 0;
	} else return 0;  /* ray is parallel to the plane of the triangle */


	inv_det = 1.0 / det;

	/* calculate t, ray intersects triangle */
	*t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;
	(*u) *= inv_det;
	(*v) *= inv_det;

	return 1;
}

/* code rewritten to do tests on the sign of the determinant */
/* the division is before the test of the sign of the det    */
inline int intersect_triangle2(double orig[3], double dir[3],
							   double vert0[3], double vert1[3], double vert2[3],
							   double *t, double *u, double *v)
{
	double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	double det, inv_det;

	/* find vectors for two edges sharing vert0 */
	IGL_RAY_TRI_SUB(edge1, vert1, vert0);
	IGL_RAY_TRI_SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	IGL_RAY_TRI_CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = IGL_RAY_TRI_DOT(edge1, pvec);

	/* calculate distance from vert0 to ray origin */
	IGL_RAY_TRI_SUB(tvec, orig, vert0);
	inv_det = 1.0 / det;

	if (det > IGL_RAY_TRI_EPSILON)
	{
		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (*u < 0.0 || *u > det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v < 0.0 || *u + *v > det)
			return 0;
	} else if (det < -IGL_RAY_TRI_EPSILON)
	{
		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (*u > 0.0 || *u < det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v > 0.0 || *u + *v < det)
			return 0;
	} else return 0;  /* ray is parallel to the plane of the triangle */

	/* calculate t, ray intersects triangle */
	*t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;
	(*u) *= inv_det;
	(*v) *= inv_det;

	return 1;
}

/* code rewritten to do tests on the sign of the determinant */
/* the division is before the test of the sign of the det    */
/* and one IGL_RAY_TRI_CROSS has been moved out from the if-else if-else */
inline int intersect_triangle3(double orig[3], double dir[3],
							   double vert0[3], double vert1[3], double vert2[3],
							   double *t, double *u, double *v)
{
	double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	double det, inv_det;

	/* find vectors for two edges sharing vert0 */
	IGL_RAY_TRI_SUB(edge1, vert1, vert0);
	IGL_RAY_TRI_SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	IGL_RAY_TRI_CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = IGL_RAY_TRI_DOT(edge1, pvec);

	/* calculate distance from vert0 to ray origin */
	IGL_RAY_TRI_SUB(tvec, orig, vert0);
	inv_det = 1.0 / det;

	IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

	if (det > IGL_RAY_TRI_EPSILON)
	{
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (*u < 0.0 || *u > det)
			return 0;

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v < 0.0 || *u + *v > det)
			return 0;
	} else if (det < -IGL_RAY_TRI_EPSILON)
	{
		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (*u > 0.0 || *u < det)
			return 0;

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v > 0.0 || *u + *v < det)
			return 0;
	} else return 0;  /* ray is parallel to the plane of the triangle */

	*t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;
	(*u) *= inv_det;
	(*v) *= inv_det;

	return 1;
}
#endif

namespace igl
{
// Shoot a ray against a mesh (V,F) and collect all hits. If you have many
// rays, consider using AABB.h
//
// Inputs:
//   source  3-vector origin of ray
//   dir  3-vector direction of ray
//   V  #V by 3 list of mesh vertex positions
//   F  #F by 3 list of mesh face indices into V
// Outputs:
//    hits  **sorted** list of hits
// Returns true if there were any hits (hits.size() > 0)
//
// See also: AABB.h
template<typename Derivedsource, typename Deriveddir, typename DerivedV, typename DerivedF>
IGL_INLINE bool ray_mesh_intersect(
		const Eigen::MatrixBase<Derivedsource> &s,
		const Eigen::MatrixBase<Deriveddir> &dir,
		const Eigen::MatrixBase<DerivedV> &V,
		const Eigen::MatrixBase<DerivedF> &F,
		std::vector<igl::Hit> &hits)
{
	using namespace Eigen;
	using namespace std;
	// Should be but can't be const
	Vector3d s_d = s.template cast<double>();
	Vector3d dir_d = dir.template cast<double>();
	hits.clear();
	hits.reserve(F.rows());

	// loop over all triangles
	for (int f = 0; f < F.rows(); f++)
	{
		// Should be but can't be const
		RowVector3d v0 = V.row(F(f, 0)).template cast<double>();
		RowVector3d v1 = V.row(F(f, 1)).template cast<double>();
		RowVector3d v2 = V.row(F(f, 2)).template cast<double>();
		// shoot ray, record hit
		double t, u, v;
		if (intersect_triangle1(
				s_d.data(), dir_d.data(), v0.data(), v1.data(), v2.data(), &t, &u, &v) &&
			t > 0)
		{
			hits.push_back({(int) f, (int) -1, (float) u, (float) v, (float) t});
		}
	}
	// Sort hits based on distance
	std::sort(
			hits.begin(),
			hits.end(),
			[](const Hit &a, const Hit &b) -> bool { return a.t < b.t; });
	return hits.size() > 0;
}


// Outputs:
//   hit  first hit, set only if it exists
// Returns true if there was a hit
template<typename Derivedsource, typename Deriveddir, typename DerivedV, typename DerivedF>
IGL_INLINE bool ray_mesh_intersect(
		const Eigen::MatrixBase<Derivedsource> &source,
		const Eigen::MatrixBase<Deriveddir> &dir,
		const Eigen::MatrixBase<DerivedV> &V,
		const Eigen::MatrixBase<DerivedF> &F,
		igl::Hit &hit)
{
	std::vector<igl::Hit> hits;
	ray_mesh_intersect(source, dir, V, F, hits);
	if (hits.size() > 0)
	{
		hit = hits.front();
		return true;
	} else
	{
		return false;
	}
}
}
#endif
