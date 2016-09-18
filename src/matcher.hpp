#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cmath>
#include "naive_mapping/types.h"
#include "map.hpp"
#include "transformer.hpp"

namespace naive_mapping
{

class Matcher
{
public:
	Beta match(const std::vector<GridMap> &maps,
			const std::vector<Point> &points, Beta initial_pose)
	{
		for (std::vector<GridMap>::const_reverse_iterator endmap =
				maps.rbegin(); endmap != maps.rend(); endmap++)
		{
			//std::cout << (*endmap).getFrame() << std::endl;
			initial_pose = iterateGuessNewton(*endmap, points, initial_pose, 3);
			//std::cout << initial_pose.transpose() << std::endl;
		}
		//initial_pose = iterateGuessNewton(maps[0], points, initial_pose, 5);

		/*initial_pose = iterateGuessNewton(maps[4], points, initial_pose, 1);
		 initial_pose = iterateGuessNewton(maps[3], points, initial_pose, 2);*/
		 /*initial_pose = iterateGuessNewton(maps[2], points, initial_pose, 3);
		 initial_pose = iterateGuessNewton(maps[1], points, initial_pose, 3);
		 initial_pose = iterateGuessNewton(maps[0], points, initial_pose, 3);*/
		//std::cout << std::endl;
		return initial_pose;
	}
private:
	Beta iterateGuessNewton(const GridMap &map,
			const std::vector<Point> &points, Beta beta, int n_iterators)
	{
		for (int i = 0; i < n_iterators; i++)
		{
			//std::cout << "iterator:" << i << std::endl;
			setHessian(map, points, beta);
			//std::cout << "residual:" << residual_ << std::endl;
			//
			//std::cout << "H:" << std::endl << H_ << std::endl;
			//std::cout << "g^T:" << g_.transpose() << std::endl;
			//
			if ((H_(0, 0) != 0.0f) && (H_(1, 1) != 0.0f))
			{
				Eigen::Vector3f delta = H_.inverse() * g_;
				//
				//std::cout << "delta^T:" << delta.transpose() << std::endl;
				//
				if (delta[2] > 0.2f)
				{
					delta[2] = 0.2f;
					std::cout << "rotation change too large" << std::endl;
				}
				else if (delta[2] < -0.2f)
				{
					delta[2] = -0.2f;
					std::cout << "rotation change too large" << std::endl;
				}
				beta = beta - delta;
				normalizeAngle(beta.theta());
			}
			else
			{
				std::cout << "H could not be inversed" << std::endl;
				break;
			}
		}
		return beta;
	}
	void setHessian(const GridMap &map, const std::vector<Point> &points,
			const Beta &beta)
	{
		float sin_yaw = sin(beta.theta());
		float cos_yaw = cos(beta.theta());

		g_ = Eigen::Vector3f::Zero();
		H_ = Eigen::Matrix3f::Zero();
		residual_ = 0;

		int i=0;
		for (std::vector<Point>::const_iterator endpoint = points.begin();
				endpoint != points.end(); endpoint++,i++)
		{
			if (1e-5 > std::abs((*endpoint).x())
					|| 1e-5 > std::abs((*endpoint).y()))
			{
				continue;
			}
			Eigen::Matrix3f trans = Transformer::toMatrix(beta);
			Point current = *endpoint;
			Eigen::Vector3f result = map.getBilinearInterpolation(
					trans * current);
			float r = 100.0f - result[0];
			residual_ += r;
			float Drot = ((result[1])
					* (-sin_yaw * current.x() - cos_yaw * current.y()))
					+ ((result[2])
							* (cos_yaw * current.x() - sin_yaw * current.y()));
			g_[0] += -result[1] * r;
			g_[1] += -result[2] * r;
			g_[2] += -Drot * r;
			//
			H_(0, 0) += result[1] * result[1] * (1 + r);
			H_(0, 1) += result[1] * result[2] * (1 + r);
			H_(0, 2) += result[1] * Drot * (1 + r);
			H_(1, 1) += result[2] * result[2] * (1 + r);
			H_(1, 2) += result[2] * Drot * (1 + r);
			H_(2, 2) += Drot * Drot * (1 + r);
		}
		H_(1, 0) = H_(0, 1);
		H_(2, 0) = H_(0, 2);
		H_(2, 1) = H_(1, 2);
	}
	void normalizeAngle(float &angle)
	{
		float a = fmod(fmod(angle, 2.0f * M_PI) + 2.0f * M_PI, 2.0f * M_PI);
		if (a > M_PI)
		{
			a -= 2.0f * M_PI;
		}
		angle = a;
	}
private:
	Eigen::Vector3f g_;
	Eigen::Matrix3f H_;
	float residual_;
};

}
