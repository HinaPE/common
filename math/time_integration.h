#ifndef HINAPE_TIME_INTEGRATION_H
#define HINAPE_TIME_INTEGRATION_H

#include "math/vector.h"
#include "math/quaternion.h"

//@formatter:on
namespace HinaPE::TimeIntegration
{
template<typename VectorType>
void semi_implicit_euler(real dt, real m, VectorType &p, VectorType &v, const VectorType &a)
{
	if (Math::similar(m, Constant::Zero, 0.00001))
		return;

	v += dt * a;
	p += dt * v;
}
} // namespace HinaPE::TimeIntegration

#endif //HINAPE_TIME_INTEGRATION_H
