#ifndef HINAPE_TRIANGLE_MESH_H
#define HINAPE_TRIANGLE_MESH_H

#include "surface3.h"
#include "bvh.h"

namespace HinaPE::Geom
{
class TriangleMeshSurface : public Surface3
{
public:



private:
	std::vector<mVector3> vertices;
	std::vector<mVector3> normals;
	std::vector<mVector2> uvs;
	std::vector<unsigned int> indices;
	std::vector<unsigned int> normal_indices;
	std::vector<unsigned int> uv_indices;
};
} // namespace HinaPE::Geom

#endif //HINAPE_TRIANGLE_MESH_H
