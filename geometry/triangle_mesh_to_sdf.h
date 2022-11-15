#ifndef HINAPE_TRIANGLE_MESH_TO_SDF_H_
#define HINAPE_TRIANGLE_MESH_TO_SDF_H_

#include "triangle_mesh3.h"

#include "grid/scalar_grid3.h"

namespace HinaPE
{

//!
//! \brief Generates signed-distance field out of given triangle mesh.
//!
//! This function generates signed-distance field from a triangle mesh. The sign
//! is determined by TriangleMesh3::isInside (negative means inside).
//!
//! \warning Parameter \p exactBand is no longer used and will be deprecated in
//! next release (v2.x).
//!
//! \param[in]      mesh      The mesh.
//! \param[in,out]  sdf       The output signed-distance field.
//! \param[in]      exactBand This parameter is no longer used.
//!
void triangleMeshToSdf(const TriangleMesh3 &mesh, ScalarGrid3 *sdf, const unsigned int exactBand = 1);

}  // namespace HinaPE

#endif  // HINAPE_TRIANGLE_MESH_TO_SDF_H_
