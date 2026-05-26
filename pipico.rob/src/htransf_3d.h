#ifndef VECTOR_H
#define VECTOR_H

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <VectorXf.h>


// convert to / from polar coordinates where r is stored in x, longitude
// is stored in y, latitude in z
inline const Vec3f to_polar(const Vec3f &pt) {
	Vec3f ret;
	ret.x = pt.length();
	ret.y = atan2(pt.y, pt.x);
	ret.z = acos(pt.z / ret.x);
	return ret;
}

inline const Vec3f from_polar(const Vec3f &pt) {
	Vec3f ret;
	ret.x = pt.x * sin(pt.z) * cos(pt.y);
	ret.y = pt.x * sin(pt.z) * sin(pt.y);
	ret.z = pt.x * cos(pt.z);
	return ret;
}

/*
void cartesian_to_polar(vector <float> a, float& r, float& lat, float& lon)
{ r = modulo(a);
lon = atan2(a[1], a[0]);
lat = acos(a[2] / r);
}

void polar_to_cartesian(float r, float lat, float lon, vector <float>& a)
{ a[2] = r * cos(lat);
 a[0] = r * sin(lat) * cos(lon);
  a[1] = r * sin(lat) * sin(lon);
}
*/


class transform3d_t {
public:
	float d[3][4];

	transform3d_t() {
		memset(d, 0, sizeof(d));
		d[0][0] = 1.0;
		d[1][1] = 1.0;
		d[2][2] = 1.0;
	}

	transform3d_t operator*(const transform3d_t &p) const {
		transform3d_t ret;
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 4; c++)
				//ret.d[r][c] = p.d[r][0] * d[0][c] + p.d[r][1] * d[1][c] + p.d[r][2] * d[2][c] + p.d[r][3] * (c == 3);
				ret.d[r][c] = p.d[0][c] * d[r][0] + p.d[1][c] * d[r][1] + p.d[2][c] * d[r][2] + (c == 3) * d[r][3]; // correct
		return ret;
	}

	Vec3f operator*(const Vec3f &p) const {
		Vec3f ret;
		ret.x = p.x * d[0][0] + p.y * d[0][1] + p.z * d[0][2] + d[0][3];
		ret.y = p.x * d[1][0] + p.y * d[1][1] + p.z * d[1][2] + d[1][3];
		ret.z = p.x * d[2][0] + p.y * d[2][1] + p.z * d[2][2] + d[2][3];
		return ret;
	}
};

inline const transform3d_t translate3d(const Vec3f &pt)
{
	transform3d_t ret;
	ret.d[0][3] = pt.x;
	ret.d[1][3] = pt.y;
	ret.d[2][3] = pt.z;
	return ret;
}

inline const transform3d_t scale3d(const Vec3f &pt)
{
	transform3d_t ret;
	ret.d[0][0] = pt.x;
	ret.d[1][1] = pt.y;
	ret.d[2][2] = pt.z;
	return ret;
}

inline const transform3d_t rot3d_x(const float angle)
{
	float ca = cos(angle), sa = sin(angle);
	transform3d_t ret;
	ret.d[1][1] = ca;
	ret.d[1][2] = -sa;
	ret.d[2][1] = sa;
	ret.d[2][2] = ca;
	return ret;
}

inline const transform3d_t rot3d_y(const float angle)
{
	float ca = cos(angle), sa = sin(angle);
	transform3d_t ret;
	ret.d[0][0] = ca;
	ret.d[0][2] = sa;
	ret.d[2][0] = -sa;
	ret.d[2][2] = ca;
	return ret;
}

inline const transform3d_t rot3d_z(const float angle)
{
	float ca = cos(angle), sa = sin(angle);
	transform3d_t ret;
	ret.d[0][0] = ca;
	ret.d[0][1] = -sa;
	ret.d[1][0] = sa;
	ret.d[1][1] = ca;
	return ret;
}

#endif
