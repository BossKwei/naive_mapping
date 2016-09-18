#pragma once
#include <stdint.h>
#include <cmath>
#include <Eigen/Eigen>

namespace naive_mapping
{

class Point: public Eigen::Vector3f
{
public:
	Point(const float &x = 0.0f, const float &y = 0.0f) :
			Eigen::Vector3f(x, y, 1.0f)
	{
	}
	/*Point(const Eigen::Vector3f &other) :
	 Eigen::Vector3f(other)
	 {
	 }*/
	/*template<typename OtherDerived>
	 EIGEN_DEVICE_FUNC
	 EIGEN_STRONG_INLINE Matrix(const EigenBase<OtherDerived> &other)
	 : Base(other.derived())
	 {}*/
	template<typename OtherDerived>
	Point(const EigenBase<OtherDerived> &other) :
			Eigen::Vector3f(other)
	{
	}
	/*template<typename OtherDerived>
	 Point& operator=(const EigenBase<OtherDerived> &other)
	 {
	 Eigen::Vector3f::operator=(other);
	 return *this;
	 }*/
	/*Point& operator=(const Eigen::Vector3f &other)
	 {
	 Eigen::Vector3f::operator=(other);
	 return *this;
	 }*/
};

typedef Eigen::Vector2i Index;

class Beta: public Eigen::Vector3f
{
public:
	Beta(const float &x, const float &y, const float &theta) :
			Eigen::Vector3f(x, y, theta)
	{
	}
	template<typename OtherDerived>
	Beta(const EigenBase<OtherDerived> &other) :
			Eigen::Vector3f(other)
	{
	}
	float& theta()
	{
		return Eigen::Vector3f::operator [](2);
	}
	float theta() const
	{
		return Eigen::Vector3f::operator [](2);
	}
};

}
