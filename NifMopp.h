#pragma once

#include <windows.h>
#include <exception>
#include <excpt.h>
#include <cstring>
#include <stdio.h>

#if _MSC_VER >= 1900
extern "C" FILE * __iob_func( void );
#endif

struct Point3
{
	float x, y, z;
	void Set(float x, float y, float z){
		this->x = x; this->y = y; this->z = z;
	}
};

struct Triangle
{
	unsigned short a, b, c;

	unsigned short operator[](size_t i) const {
		switch (i)
		{
		case 0: return a;
		case 1: return b;
		case 2: return c;
		default: throw std::exception("Invalid index");
		}
	}
	unsigned short& operator[](size_t i) {
		switch (i)
		{
		case 0: return a;
		case 1: return b;
		case 2: return c;
		default: throw std::exception("Invalid index");
		}
	}
};

struct Matrix3
{
	float m[3][3];
};

struct Matrix43
{
	float m[4][3];
};

struct Matrix44
{
	float m[4][4];
};

extern void InitializeHavok();
