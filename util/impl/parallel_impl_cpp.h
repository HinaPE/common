#ifndef HINAPE_PARALLEL_IMPL_CPP_H
#define HINAPE_PARALLEL_IMPL_CPP_H

#include <thread>

#include "../parallel.h"

namespace HinaPE::Util
{
template<typename IndexType, typename Function>
void parallelFor(IndexType begin_index, IndexType end_index, const Function &function, ExecutionPolicy policy)
{
	if (begin_index > end_index)
		return;

	for (IndexType i = begin_index; i < end_index; ++i)
		function(i);
}

template<typename IndexType, typename Function>
void parallelRangeFor(IndexType begin_index, IndexType end_index, const Function &function, ExecutionPolicy policy)
{
	if (begin_index > end_index)
		return;

	function(begin_index, end_index);
}

template<typename IndexType, typename Function>
void parallelFor(IndexType begin_indexX, IndexType end_indexX, IndexType begin_indexY, IndexType end_indexY, const Function &function, ExecutionPolicy policy)
{
	parallelFor(begin_indexY, end_indexY,
				[&](IndexType j)
				{
					for (IndexType i = begin_indexX; i < end_indexX; ++i)
						function(i, j);
				},
				policy);
}

template<typename IndexType, typename Function>
void parallelRangeFor(IndexType begin_indexX, IndexType end_indexX, IndexType begin_indexY, IndexType end_indexY, const Function &function, ExecutionPolicy policy)
{
	parallelRangeFor(begin_indexY, end_indexY,
					 [&](IndexType jBegin, IndexType jEnd)
					 {
						 function(begin_indexX, end_indexX, jBegin, jEnd);
					 },
					 policy);
}

template<typename IndexType, typename Function>
void parallelFor(IndexType begin_indexX, IndexType end_indexX, IndexType begin_indexY, IndexType end_indexY, IndexType begin_indexZ, IndexType end_indexZ, const Function &function, ExecutionPolicy policy)
{
	parallelFor(begin_indexZ, end_indexZ,
				[&](IndexType k)
				{
					for (IndexType j = begin_indexY; j < end_indexY; ++j)
						for (IndexType i = begin_indexX; i < end_indexX; ++i)
							function(i, j, k);
				},
				policy);
}

template<typename IndexType, typename Function>
void parallelRangeFor(IndexType begin_indexX, IndexType end_indexX, IndexType begin_indexY, IndexType end_indexY, IndexType begin_indexZ, IndexType end_indexZ, const Function &function, ExecutionPolicy policy)
{
	parallelRangeFor(begin_indexZ, end_indexZ,
					 [&](IndexType kBegin, IndexType kEnd)
					 {
						 function(begin_indexX, end_indexX, begin_indexY, end_indexY, kBegin, kEnd);
					 },
					 policy);
}

template<typename RandomIterator, typename T>
void parallelFill(const RandomIterator &begin, const RandomIterator &end, const T &value, ExecutionPolicy policy) {}
template<typename IndexType, typename Value, typename Function, typename Reduce>
auto parallelReduce(IndexType begin_index, IndexType end_index, const Value &identity, const Function &func, const Reduce &reduce, ExecutionPolicy) -> Value {}
template<typename RandomIterator>
void parallelSort(RandomIterator begin, RandomIterator end, ExecutionPolicy policy) {}
template<typename RandomIterator, typename CompareFunction>
void parallelSort(RandomIterator begin, RandomIterator end, CompareFunction compare, ExecutionPolicy policy) {}
} // namespace HinaPE::Util
#endif //HINAPE_PARALLEL_IMPL_CPP_H
