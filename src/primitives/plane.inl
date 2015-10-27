__device__ __host__
inline float	Plane::f(const Point_cu& p) const {
	float d = fmaxf(0.f, pt_vec_dot(p, n) - offset);
	return Field::distance_to_field(d, radius);
}

__device__ __host__
inline Vec3_cu	Plane::gf(const Point_cu& p) const {
	float d = fmaxf(0.f,pt_vec_dot(p, n) - offset);
	float df = Field::field_derivative_from_distance(d, radius);
	return n * df;
}

__device__ __host__
inline float Plane::fngf(Vec3_cu& gf, const Point_cu& p) const {
	float d = fmaxf(0.f,pt_vec_dot(p, n) - offset);
	float2 fndf = Field::distance_to_field_and_derivative(d, radius);
	gf = n * fndf.y;
	return fndf.x;
}


__device__ __host__
inline float Plane::pt_vec_dot(const Point_cu& p, const Vec3_cu& v){
	return p.x * v.x + p.y * v.y + p.z * v.z;
}

