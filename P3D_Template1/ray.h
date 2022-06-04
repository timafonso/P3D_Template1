#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
	Ray(const Vector& o, const Vector& dir ) : origin(o), direction(dir), time(0.0f) {};
	Ray(const Vector& o, const Vector& dir,  float time) : origin(o), direction(dir), time(time) {};

	Vector origin;
	Vector direction;
	float time;
};
#endif