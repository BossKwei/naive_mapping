#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Eigen>
#include "naive_mapping/types.h"

namespace naive_mapping
{

class Geometry
{
public:
	Geometry() :
			angle_min_(0.0f), angle_max_(0.0f)
	{
	}

	void projectLaser(const sensor_msgs::LaserScan& scan_in,
			std::vector<Point> &points_out, const float range_cutoff = 30.0f)
	{
		size_t n_points = scan_in.ranges.size();

		if (co_sine_map_.rows() != n_points || angle_min_ != scan_in.angle_min
				|| angle_max_ != scan_in.angle_max)
		{
			ROS_INFO("Precomputed map built");

			co_sine_map_ = Eigen::ArrayXXf(n_points, 2);
			angle_min_ = scan_in.angle_min;
			angle_max_ = scan_in.angle_max;

			for (size_t i = 0; i < n_points; ++i)
			{
				co_sine_map_(i, 0) =
						cos(
								angle_min_
										+ static_cast<float>(i
												* scan_in.angle_increment));
				co_sine_map_(i, 1) =
						sin(
								angle_min_
										+ static_cast<float>(i
												* scan_in.angle_increment));
			}
		}

		if (points_out.size() != n_points)
		{
			ROS_INFO("Resize points container");
			points_out.clear();
			points_out.resize(n_points);
		}

		float range;
		for (size_t i = 0; i < n_points; i++)
		{
			if(std::isinf(scan_in.ranges[i]))
			{
				range = 0;
			}
			else if(std::isnan(scan_in.ranges[i]))
			{
				ROS_ERROR("Laser data has a nan value!");
			}
			else
			{
				range = scan_in.ranges[i] > range_cutoff ?
									range_cutoff : scan_in.ranges[i];
			}

			points_out[i].x() = range * co_sine_map_(i, 0);
			points_out[i].y() = range * co_sine_map_(i, 1);
		}

		return;
	}

private:
	Eigen::ArrayXXf co_sine_map_;
	float angle_min_;
	float angle_max_;
};

}
