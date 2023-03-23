#include "../grid.h"

auto generate_indices_and_weights(const mVector3 &x, const mVector3 &origin, const mVector3 &spacing, const mSize3 size) -> std::tuple<std::array<mSize3, 8>, std::array<real, 8>>
{
	std::array<mSize3, 8> indices{};
	std::array<real, 8> weights{};

	long i, j, k;
	real fx, fy, fz;

	const mVector3 normalized_x = (x - origin) / spacing;

	const long i_size = static_cast<long>(size.x);
	const long j_size = static_cast<long>(size.y);
	const long k_size = static_cast<long>(size.z);

	HinaPE::Math::get_barycentric(normalized_x.x(), 0, i_size - 1, &i, &fx);
	HinaPE::Math::get_barycentric(normalized_x.y(), 0, j_size - 1, &j, &fy);
	HinaPE::Math::get_barycentric(normalized_x.z(), 0, k_size - 1, &k, &fz);

	const long ip1 = std::min(i + 1, i_size - 1);
	const long jp1 = std::min(j + 1, j_size - 1);
	const long kp1 = std::min(k + 1, k_size - 1);

	indices[0] = mSize3(i, j, k);
	indices[1] = mSize3(ip1, j, k);
	indices[2] = mSize3(i, jp1, k);
	indices[3] = mSize3(ip1, jp1, k);
	indices[4] = mSize3(i, j, kp1);
	indices[5] = mSize3(ip1, j, kp1);
	indices[6] = mSize3(i, jp1, kp1);
	indices[7] = mSize3(ip1, jp1, kp1);

	weights[0] = (1 - fx) * (1 - fy) * (1 - fz);
	weights[1] = fx * (1 - fy) * (1 - fz);
	weights[2] = (1 - fx) * fy * (1 - fz);
	weights[3] = fx * fy * (1 - fz);
	weights[4] = (1 - fx) * (1 - fy) * fz;
	weights[5] = fx * (1 - fy) * fz;
	weights[6] = (1 - fx) * fy * fz;
	weights[7] = fx * fy * fz;

	return std::make_tuple(indices, weights);
}

auto HinaPE::Geom::ScalarGridField3::gradient(const mVector3 &x, HinaPE::Geom::DataGrid3<real>::Data data) const -> mVector3
{
	auto *target = &data_center;
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

	auto size = target->size();
	auto res = generate_indices_and_weights(x, origin, spacing, size);
	std::array<mSize3, 8> indices = std::get<0>(res);
	std::array<real, 8> weights = std::get<1>(res);

	mVector3 result;
	for (size_t s = 0; s < 8; ++s)
		result += weights[s] * gradient_at_data_point(indices[s].x, indices[s].y, indices[s].z, data);

	return result;
}
auto HinaPE::Geom::ScalarGridField3::laplacian(const mVector3 &x, HinaPE::Geom::DataGrid3<real>::Data data) const -> real
{
	auto *target = &data_center;
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

	auto size = target->size();
	auto res = generate_indices_and_weights(x, origin, spacing, size);
	std::array<mSize3, 8> indices = std::get<0>(res);
	std::array<real, 8> weights = std::get<1>(res);

	real result;
	for (size_t s = 0; s < 8; ++s)
		result += weights[s] * laplacian_at_data_point(indices[s].x, indices[s].y, indices[s].z, data);

	return result;
}
auto HinaPE::Geom::ScalarGridField3::gradient_at_data_point(size_t i, size_t j, size_t k, HinaPE::Geom::DataGrid3<real>::Data data) const -> mVector3
{
	auto *target = &data_center;
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

	auto size = target->size();

	const real left = (*target)((i > 0) ? i - 1 : i, j, k);
	const real right = (*target)((i + 1 < size.x) ? i + 1 : i, j, k);
	const real down = (*target)(i, (j > 0) ? j - 1 : j, k);
	const real up = (*target)(i, (j + 1 < size.y) ? j + 1 : j, k);
	const real back = (*target)(i, j, (k > 0) ? k - 1 : k);
	const real front = (*target)(i, j, (k + 1 < size.z) ? k + 1 : k);

	return {(right - left) / spacing.x(),
			(up - down) / spacing.y(),
			(front - back) / spacing.z()};
}
auto HinaPE::Geom::ScalarGridField3::laplacian_at_data_point(size_t i, size_t j, size_t k, HinaPE::Geom::DataGrid3<real>::Data data) const -> real
{
	auto *target = &data_center;
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

	auto size = target->size();
	auto c = (*target)(i, j, k);

	real d_left = 0;
	real d_right = 0;
	real d_down = 0;
	real d_up = 0;
	real d_back = 0;
	real d_front = 0;

	if (i > 0)
		d_left = c - (*target)(i - 1, j, k);
	if (i < size.x - 1)
		d_right = (*target)(i + 1, j, k) - c;
	if (j > 0)
		d_down = c - (*target)(i, j - 1, k);
	if (j < size.y - 1)
		d_up = (*target)(i, j + 1, k) - c;
	if (k > 0)
		d_back = c - (*target)(i, j, k - 1);
	if (k < size.z - 1)
		d_front = (*target)(i, j, k + 1) - c;

	return (d_left + d_right) / (spacing.x() * spacing.x()) +
		   (d_down + d_up) / (spacing.y() * spacing.y()) +
		   (d_back + d_front) / (spacing.z() * spacing.z());
}

auto HinaPE::Geom::VectorGridField3::divergence(const mVector3 &x, HinaPE::Geom::DataGrid3<mVector3>::Data data) const -> real
{
	auto *target = &data_center;
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

	auto size = target->size();
	auto res = generate_indices_and_weights(x, origin, spacing, size);
	std::array<mSize3, 8> indices = std::get<0>(res);
	std::array<real, 8> weights = std::get<1>(res);

	real result = 0;
	for (size_t s = 0; s < 8; ++s)
		result += weights[s] * divergence_at_data_point(indices[s].x, indices[s].y, indices[s].z, data);

	return result;
}
auto HinaPE::Geom::VectorGridField3::curl(const mVector3 &x, HinaPE::Geom::DataGrid3<mVector3>::Data data) const -> mVector3
{
	auto *target = &data_center;
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

	auto size = target->size();
	auto res = generate_indices_and_weights(x, origin, spacing, size);
	std::array<mSize3, 8> indices = std::get<0>(res);
	std::array<real, 8> weights = std::get<1>(res);

	mVector3 result = {0, 0, 0};
	for (size_t s = 0; s < 8; ++s)
		result += weights[s] * curl_at_data_point(indices[s].x, indices[s].y, indices[s].z, data);

	return result;
}
auto HinaPE::Geom::VectorGridField3::divergence_at_data_point(size_t i, size_t j, size_t k, HinaPE::Geom::DataGrid3<mVector3>::Data data) const -> real
{
	auto *target = &data_center;
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

	auto size = target->size();

	real left = (*target)((i > 0) ? i - 1 : i, j, k).x();
	real right = (*target)((i < size.x - 1) ? i + 1 : i, j, k).x();
	real down = (*target)(i, (j > 0) ? j - 1 : j, k).y();
	real up = (*target)(i, (j < size.y - 1) ? j + 1 : j, k).y();
	real back = (*target)(i, j, (k > 0) ? k - 1 : k).z();
	real front = (*target)(i, j, (k < size.z - 1) ? k + 1 : k).z();

	return HinaPE::Constant::Half * (right - left) / spacing.x() +
		   HinaPE::Constant::Half * (up - down) / spacing.y() +
		   HinaPE::Constant::Half * (front - back) / spacing.z();
}
auto HinaPE::Geom::VectorGridField3::curl_at_data_point(size_t i, size_t j, size_t k, HinaPE::Geom::DataGrid3<mVector3>::Data data) const -> mVector3
{
	auto *target = &data_center;
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

	auto size = target->size();

	mVector3 left = (*target)((i > 0) ? i - 1 : i, j, k);
	mVector3 right = (*target)((i < size.x - 1) ? i + 1 : i, j, k);
	mVector3 down = (*target)(i, (j > 0) ? j - 1 : j, k);
	mVector3 up = (*target)(i, (j < size.y - 1) ? j + 1 : j, k);
	mVector3 back = (*target)(i, j, (k > 0) ? k - 1 : k);
	mVector3 front = (*target)(i, j, (k < size.z - 1) ? k + 1 : k);

	real Fx_ym = down.x();
	real Fx_yp = up.x();
	real Fx_zm = back.x();
	real Fx_zp = front.x();

	real Fy_xm = left.y();
	real Fy_xp = right.y();
	real Fy_zm = back.y();
	real Fy_zp = front.y();

	real Fz_xm = left.z();
	real Fz_xp = right.z();
	real Fz_ym = down.z();
	real Fz_yp = up.z();

	return mVector3{
		HinaPE::Constant::Half * (Fz_yp - Fz_ym) / spacing.y() - HinaPE::Constant::Half * (Fy_zp - Fy_zm) / spacing.z(),
		HinaPE::Constant::Half * (Fx_zp - Fx_zm) / spacing.z() - HinaPE::Constant::Half * (Fz_xp - Fz_xm) / spacing.x(),
		HinaPE::Constant::Half * (Fy_xp - Fy_xm) / spacing.x() - HinaPE::Constant::Half * (Fx_yp - Fx_ym) / spacing.y()
	};
}
