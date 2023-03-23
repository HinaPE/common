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
struct DataGrid3
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

	enum class Data
	{
		Center,
		Vertex,
		FaceU,
		FaceV,
		FaceW
	};

	DataGrid3();
	void resize(const mSize3 &resolution, const mVector3 &spacing, const mVector3 &center = mVector3::Zero());
	auto bbox() const -> mBBox3;

	auto sample(const mVector3 &x, Data data = Data::Center) const -> real;
	auto cell(size_t x, size_t y, size_t z) const -> Cell;
	auto pos_center(size_t x, size_t y, size_t z) const -> mVector3;
	auto pos_vertex(size_t x, size_t y, size_t z) const -> mVector3;
	auto pos_face_u(size_t x, size_t y, size_t z) const -> mVector3;
	auto pos_face_v(size_t x, size_t y, size_t z) const -> mVector3;
	auto pos_face_w(size_t x, size_t y, size_t z) const -> mVector3;
};

template<typename T>
DataGrid3<T>::DataGrid3()
{
	resize(mSize3::Ones(), mVector3::One());
}

template<typename T>
void DataGrid3<T>::resize(const mSize3 &r, const mVector3 &s, const mVector3 &c)
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
auto DataGrid3<T>::bbox() const -> mBBox3
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

template<typename T>
auto DataGrid3<T>::sample(const mVector3 &x, DataGrid3::Data data) const -> real
{
	// linear interpolation
	long i, j, k;
	real fx, fy, fz;
	mVector3 normalized_x = (x - origin) / spacing;

	auto *target = &data_center;

	size_t i_size = data_center.size().x;
	size_t j_size = data_center.size().y;
	size_t k_size = data_center.size().z;

	Math::get_barycentric(normalized_x.x(), 0, static_cast<long>(i_size) - 1, &i, &fx);
	Math::get_barycentric(normalized_x.y(), 0, static_cast<long>(j_size) - 1, &j, &fy);
	Math::get_barycentric(normalized_x.z(), 0, static_cast<long>(k_size) - 1, &k, &fz);

	long ip1 = std::min(i + 1, static_cast<long>(i_size) - 1);
	long jp1 = std::min(j + 1, static_cast<long>(j_size) - 1);
	long kp1 = std::min(k + 1, static_cast<long>(k_size) - 1);

	switch (data)
	{
		case Data::Center:
			target = &data_center;
			break;
		case Data::Vertex:
			target = &data_vertex;
			break;
		case Data::FaceU:
			target = &data_face_u;
			break;
		case Data::FaceV:
			target = &data_face_v;
			break;
		case Data::FaceW:
			target = &data_face_w;
			break;
	}

	return Math::trilerp(
			(*target)(i, j, k),
			(*target)(ip1, j, k),
			(*target)(i, jp1, k),
			(*target)(ip1, jp1, k),
			(*target)(i, j, kp1),
			(*target)(ip1, j, kp1),
			(*target)(i, jp1, kp1),
			(*target)(ip1, jp1, kp1),
			fx, fy, fz
	);
}

template<typename T>
auto DataGrid3<T>::cell(size_t x, size_t y, size_t z) const -> DataGrid3::Cell
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
auto DataGrid3<T>::pos_center(size_t x, size_t y, size_t z) const -> mVector3
{
	if (x > (resolution.x - 1) || y > (resolution.y - 1) || z > (resolution.z - 1))
		throw std::out_of_range("DataGrid3::pos_center");

	return origin + mVector3(
			(static_cast<real>(x) + Constant::Half) * spacing.x(),
			(static_cast<real>(y) + Constant::Half) * spacing.y(),
			(static_cast<real>(z) + Constant::Half) * spacing.z()
	);
}
template<typename T>
auto DataGrid3<T>::pos_vertex(size_t x, size_t y, size_t z) const -> mVector3
{
	if (x > resolution.x || y > resolution.y || z > resolution.z)
		throw std::out_of_range("DataGrid3::pos_vertex");

	return origin + mVector3(
			static_cast<real>(x) * spacing.x(),
			static_cast<real>(y) * spacing.y(),
			static_cast<real>(z) * spacing.z()
	);
}
template<typename T>
auto DataGrid3<T>::pos_face_u(size_t x, size_t y, size_t z) const -> mVector3
{
	if (x > resolution.x || y > (resolution.y - 1) || z > (resolution.z - 1))
		throw std::out_of_range("DataGrid3::pos_face_u");

	return origin + mVector3(
			static_cast<real>(x) * spacing.x(),
			(static_cast<real>(y) + Constant::Half) * spacing.y(),
			(static_cast<real>(z) + Constant::Half) * spacing.z()
	);
}
template<typename T>
auto DataGrid3<T>::pos_face_v(size_t x, size_t y, size_t z) const -> mVector3
{
	if (x > (resolution.x - 1) || y > resolution.y || z > (resolution.z - 1))
		throw std::out_of_range("DataGrid3::pos_face_v");

	return origin + mVector3(
			(static_cast<real>(x) + Constant::Half) * spacing.x(),
			static_cast<real>(y) * spacing.y(),
			(static_cast<real>(z) + Constant::Half) * spacing.z()
	);
}
template<typename T>
auto DataGrid3<T>::pos_face_w(size_t x, size_t y, size_t z) const -> mVector3
{
	if (x > (resolution.x - 1) || y > (resolution.y - 1) || z > resolution.z)
		throw std::out_of_range("DataGrid3::pos_face_w");

	return origin + mVector3(
			(static_cast<real>(x) + Constant::Half) * spacing.x(),
			(static_cast<real>(y) + Constant::Half) * spacing.y(),
			static_cast<real>(z) * spacing.z()
	);
}

struct ScalarGridField3 : public DataGrid3<real>
{
	auto gradient(const mVector3 &x, Data data = Data::Center) const -> mVector3;
	auto laplacian(const mVector3 &x, Data data = Data::Center) const -> real;

	auto gradient_at_data_point(size_t i, size_t j, size_t k, Data data = Data::Center) const -> mVector3;
	auto laplacian_at_data_point(size_t i, size_t j, size_t k, Data data = Data::Center) const -> real;
};

struct VectorGridField3 : public DataGrid3<mVector3>
{
	auto divergence(const mVector3 &x, Data data = Data::Center) const -> real;
	auto curl(const mVector3 &x, Data data = Data::Center) const -> mVector3;

	auto divergence_at_data_point(size_t i, size_t j, size_t k, Data data = Data::Center) const -> real;
	auto curl_at_data_point(size_t i, size_t j, size_t k, Data data = Data::Center) const -> mVector3;
};
}

#endif //HINAPE_GRID_H
