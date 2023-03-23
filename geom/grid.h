#ifndef HINAPE_GRID_H
#define HINAPE_GRID_H

// Copyright (c) 2023 Xayah Hina
// MPL-2.0 license

#include "field.h"
#include "bbox.h"
#include "math/size.h"
#include "math/array.h"

namespace HinaPE::Geom
{
template<typename T>
struct ValuedGrid3
{
	Math::Array3<T> data_center;
	Math::Array3<T> data_vertex;
	Math::Array3<T> data_face_u;
	Math::Array3<T> data_face_v;
	Math::Array3<T> data_face_w;

	mSize3 resolution;
	mVector3 center;
	mVector3 spacing;
	mVector3 origin;

	struct Cell
	{
		// center data
		T center;

		// face data
		T top;
		T bottom;
		T left;
		T right;
		T front;
		T back;

		// vertex data
		T top_left_front;
		T top_right_front;
		T bottom_left_front;
		T bottom_right_front;
		T top_left_back;
		T top_right_back;
		T bottom_left_back;
		T bottom_right_back;
	};

	ValuedGrid3();
	~ValuedGrid3() = default;
	auto cell(size_t x, size_t y, size_t z) const -> Cell;
	void resize(const mSize3 &resolution, const mVector3 &spacing, const mVector3 &center = mVector3::Zero());
	auto bbox() const -> mBBox3;
};

template<typename T>
ValuedGrid3<T>::ValuedGrid3()
{
	resize(mSize3::Ones(), mVector3::One());
}

template<typename T>
void ValuedGrid3<T>::resize(const mSize3 &r, const mVector3 &s, const mVector3 &c)
{
	resolution = r;
	spacing = s;
	origin = {
			center.x() - static_cast<real>(resolution.x) * spacing.x() / 2.0,
			center.y() - static_cast<real>(resolution.y) * spacing.y() / 2.0,
			center.z() - static_cast<real>(resolution.z) * spacing.z() / 2.0
	};

	data_center.resize(resolution, T());
	data_vertex.resize(resolution + mSize3::Ones(), T());
	data_face_u.resize(resolution + mSize3(1, 0, 0), T());
	data_face_v.resize(resolution + mSize3(0, 1, 0), T());
	data_face_w.resize(resolution + mSize3(0, 0, 1), T());
}

template<typename T>
auto ValuedGrid3<T>::cell(size_t x, size_t y, size_t z) const -> ValuedGrid3::Cell
{
	// @formatter:off
	Cell cell;
	cell.center 			= data_center(x, y, z);
	cell.top 				= data_face_v(x, y + 1, z);
	cell.bottom 			= data_face_v(x, y, z);
	cell.left 				= data_face_u(x, y, z);
	cell.right 				= data_face_u(x + 1, y, z);
	cell.front 				= data_face_w(x, y, z);
	cell.back 				= data_face_w(x, y, z + 1);
	cell.top_left_front 	= data_vertex(x, y + 1, z);
	cell.top_right_front 	= data_vertex(x + 1, y + 1, z);
	cell.bottom_left_front 	= data_vertex(x, y, z);
	cell.bottom_right_front = data_vertex(x + 1, y, z);
	cell.top_left_back 		= data_vertex(x, y + 1, z + 1);
	cell.top_right_back 	= data_vertex(x + 1, y + 1, z + 1);
	cell.bottom_left_back 	= data_vertex(x, y, z + 1);
	cell.bottom_right_back 	= data_vertex(x + 1, y, z + 1);
	return cell;
	// @formatter:on
}

template<typename T>
auto ValuedGrid3<T>::bbox() const -> mBBox3
{
	mBBox3 bbox;

	bbox._lower_corner = origin;
	bbox._upper_corner = origin + mVector3(
			static_cast<real>(resolution.x) * spacing.x(),
			static_cast<real>(resolution.y) * spacing.y(),
			static_cast<real>(resolution.z) * spacing.z()
	);

	return bbox;
}
}

#endif //HINAPE_GRID_H
