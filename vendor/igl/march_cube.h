// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2021 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_MARCH_CUBE_H
#define IGL_MARCH_CUBE_H
#include "igl_inline.h"
#include <Eigen/Core>
#include <unordered_map>
namespace igl
{
// Process a single cube of a marching cubes grid.
//
// Inputs:
//   GV  #GV by 3 list of grid vertex positions
//   cS  list of 8 scalar field values at grid corners
//   cI  list of 8 indices of corners into rows of GV
//   isovalue  level-set value being extracted (often 0)
//   V  #V by 3 current list of output mesh vertex positions
//   n  current number of mesh vertices (i.e., occupied rows in V)
//   F  #F by 3 current list of output mesh triangle indices into rows of V
//   m  current number of mesh triangles (i.e., occupied rows in F)
//   E2V  current edge (GV_i,GV_j) to vertex (V_k) map
// Side-effects: V,n,F,m,E2V are updated to contain new vertices and faces of
// any constructed mesh elements
//
template<
		typename DerivedGV,
		typename Scalar,
		typename Index,
		typename DerivedV,
		typename DerivedF>
IGL_INLINE void march_cube(
		const DerivedGV &GV,
		const Eigen::Matrix<Scalar, 8, 1> &cS,
		const Eigen::Matrix<Index, 8, 1> &cI,
		const Scalar &isovalue,
		Eigen::PlainObjectBase<DerivedV> &V,
		Index &n,
		Eigen::PlainObjectBase<DerivedF> &F,
		Index &m,
		std::unordered_map<int64_t, int> &E2V)
{

// These consts get stored reasonably
#include "marching_cubes_tables.h"

	// Seems this is also successfully inlined
	const auto ij2vertex =
			[&E2V, &V, &n, &GV]
					(const Index &i, const Index &j, const Scalar &t) -> Index
			{
				// Seems this is successfully inlined.
				const auto ij2key = [](int32_t i, int32_t j)
				{
					if (i > j) { std::swap(i, j); }
					std::int64_t ret = 0;
					ret |= i;
					ret |= static_cast<std::int64_t>(j) << 32;
					return ret;
				};
				const auto key = ij2key(i, j);
				const auto it = E2V.find(key);
				int v = -1;
				if (it == E2V.end())
				{
					// new vertex
					if (n == V.rows()) { V.conservativeResize(V.rows() * 2 + 1, V.cols()); }
					V.row(n) = GV.row(i) + t * (GV.row(j) - GV.row(i));
					v = n;
					E2V[key] = v;
					n++;
				} else
				{
					v = it->second;
				}
				return v;
			};

	int c_flags = 0;
	for (int c = 0; c < 8; c++)
	{
		if (cS(c) > isovalue) { c_flags |= 1 << c; }
	}
	//Find which edges are intersected by the surface
	int e_flags = aiCubeEdgeFlags[c_flags];
	//If the cube is entirely inside or outside of the surface, then there will be no intersections
	if (e_flags == 0) { return; }
	//Find the point of intersection of the surface with each edge
	//Then find the normal to the surface at those points
	Eigen::Matrix<Index, 12, 1> edge_vertices;
	for (int e = 0; e < 12; e++)
	{
#ifndef NDEBUG
		edge_vertices[e] = -1;
#endif
		//if there is an intersection on this edge
		if (e_flags & (1 << e))
		{
			// find crossing point assuming linear interpolation along edges
			const Scalar &a = cS(a2eConnection[e][0]);
			const Scalar &b = cS(a2eConnection[e][1]);
			Scalar t;
			{
				const Scalar delta = b - a;
				if (delta == 0) { t = 0.5; }
				t = (isovalue - a) / delta;
			};
			// record global index into local table
			edge_vertices[e] =
					ij2vertex(cI(a2eConnection[e][0]), cI(a2eConnection[e][1]), t);
			assert(edge_vertices[e] >= 0);
			assert(edge_vertices[e] < n);
		}
	}
	// Insert the triangles that were found.  There can be up to five per cube
	for (int f = 0; f < 5; f++)
	{
		if (a2fConnectionTable[c_flags][3 * f] < 0) break;
		if (m == F.rows()) { F.conservativeResize(F.rows() * 2 + 1, F.cols()); }
		assert(edge_vertices[a2fConnectionTable[c_flags][3 * f + 0]] >= 0);
		assert(edge_vertices[a2fConnectionTable[c_flags][3 * f + 1]] >= 0);
		assert(edge_vertices[a2fConnectionTable[c_flags][3 * f + 2]] >= 0);
		F.row(m) <<
				 edge_vertices[a2fConnectionTable[c_flags][3 * f + 0]],
				edge_vertices[a2fConnectionTable[c_flags][3 * f + 1]],
				edge_vertices[a2fConnectionTable[c_flags][3 * f + 2]];
		m++;
	}
}
}
#endif
