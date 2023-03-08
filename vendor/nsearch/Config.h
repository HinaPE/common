#pragma once

#include "../math/math_ext.h"

namespace CompactNSearch
{
using Real = real;
}

#define INITIAL_NUMBER_OF_INDICES   50
#define INITIAL_NUMBER_OF_NEIGHBORS 50

#ifdef _MSC_VER
	#include <ppl.h>
#elif defined(__APPLE__) && defined(__clang__)
	#include <oneapi/dpl/execution>
	#include <oneapi/dpl/algorithm>
#else
	#include <parallel/algorithm>
#endif
