#ifndef HINAPE_EXPERIMENTALS_H
#define HINAPE_EXPERIMENTALS_H

#include "common/common.h"

namespace HinaPE::Util
{
template<typename T>
auto test(T a) -> T { return a; }

auto sphere_particles() -> std::vector<mVector3>;
}

#endif //HINAPE_EXPERIMENTALS_H
