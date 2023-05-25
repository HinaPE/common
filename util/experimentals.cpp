#include "experimentals.h"

auto HinaPE::Util::sphere_particles() -> std::vector<mVector3>
{
	std::vector<mVector3> positions;

	for (int i = 0; i < 100; ++i)
	{
//		std::random_device rd;
//		std::mt19937 gen(rd());
//		std::uniform_real_distribution<float> dis(0.0f, 1.0f);
//		real theta = 2.0f * M_PI * dis(gen);
//		real phi = acos(1.0f - 2.0f * dis(gen));
//		real x = sin(phi) * cos(theta);
//		real y = sin(phi) * sin(theta);
//		real z = cos(phi);
//		positions.emplace_back(x, y, z);
	}

	return positions;
}