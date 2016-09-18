#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include "naive_mapping/types.h"

namespace naive_mapping
{
namespace Transformer
{

inline Eigen::Matrix3f EulerAngle2TransformMatrix_2D(const float &x,
		const float &y, const float &yaw)
{
	float sin_yaw = sin(yaw);
	float cos_yaw = cos(yaw);
	Eigen::Matrix3f transform;
	transform << cos_yaw, -sin_yaw, x, sin_yaw, cos_yaw, y, 0.0f, 0.0f, 1.0f;
	return transform;
}

Eigen::Matrix3f toMatrix(const tf::Transform &pose)
{
	float x = pose.getOrigin().x();
	float y = pose.getOrigin().y();
	tfScalar roll, pitch, yaw;
	pose.getBasis().getRPY(roll, pitch, yaw);
	//
	return EulerAngle2TransformMatrix_2D(x, y, yaw);
}

Eigen::Matrix3f toMatrix(const Beta &beta)
{
	return EulerAngle2TransformMatrix_2D(beta.x(), beta.y(), beta.theta());
}

inline Beta toBeta(const tf::Transform &pose)
{
	float x = pose.getOrigin().x();
	float y = pose.getOrigin().y();
	tfScalar roll, pitch, yaw;
	pose.getBasis().getRPY(roll, pitch, yaw);
	//
	return Beta(x, y, yaw);
}

inline tf::Transform toTf(const Beta &beta)
{
	tf::Transform tf;
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, beta.theta());
	tf.setRotation(q);
	tf.setOrigin(tf::Vector3(beta.x(), beta.y(), 0.0));
	return tf;
}

}
}
