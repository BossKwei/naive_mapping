#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>
#include <boost/thread.hpp>

#include "naive_mapping/types.h"
#include "geometry.hpp"
#include "map.hpp"
#include "matcher.hpp"
#include "transformer.hpp"

using namespace naive_mapping;
using namespace std;

class Node
{
public:
	Node() :
			map_publish_thread_(0), estimate_pose_(0, 0, 0), last_draw_pose_(0,
					0, 0), map_is_empty_(true)
	{
		tf::Quaternion q;
		q.setRPY(0.0, 0.0, 0.0);
		tf_pose_.setRotation(q);
		tf_pose_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		//
		maps_container_.push_back(GridMap());
		maps_container_.push_back(GridMap("map2", maps_container_[0]));
		maps_container_.push_back(GridMap("map4", maps_container_[1]));
		//maps_container_.push_back(GridMap("map8", maps_container_[2]));
		//maps_container_.push_back(GridMap("map16", maps_container_[3]));
	}

	void init(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle nh;
		scan_subscriber_ = nh.subscribe("scan", 1000, &Node::scanCallback,
				this);
		map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
		//
		map_publish_thread_ = new boost::thread(
				boost::bind(&Node::publishMapLoop, this, 0.5));
		//
		ROS_INFO("Everything ready!");
		ros::spin();
	}
	void publishMapLoop(double period)
	{
		ros::Rate rate(1.0f / period);
		cv::Mat img(1024, 1024, CV_8U, 100), img2(512, 512, CV_8U, 100), img4(
				256, 256, CV_8U, 100);
		while (ros::ok())
		{
			for (int y = 0; y < 1024; y++)
			{
				for (int x = 0; x < 1024; x++)
				{
					if (maps_container_[0].getCell(Index(x, y)).isOccupied())
					{
						img.at<uint8_t>(cv::Point(x, y)) = 0;
					}
					else if (maps_container_[0].getCell(Index(x, y)).isFree())
					{
						img.at<uint8_t>(cv::Point(x, y)) = 255;
					}
				}
			}
			/*for (int y = 0; y < 512; y++)
			{
				for (int x = 0; x < 512; x++)
				{
					if (maps_container_[1].getCell(Index(x, y)).isOccupied())
					{
						img2.at<uint8_t>(cv::Point(x, y)) = 0;
					}
					else if (maps_container_[1].getCell(Index(x, y)).isFree())
					{
						img2.at<uint8_t>(cv::Point(x, y)) = 255;
					}
				}
			}
			for (int y = 0; y < 256; y++)
			{
				for (int x = 0; x < 256; x++)
				{
					if (maps_container_[2].getCell(Index(x, y)).isOccupied())
					{
						img4.at<uint8_t>(cv::Point(x, y)) = 0;
					}
					else if (maps_container_[2].getCell(Index(x, y)).isFree())
					{
						img4.at<uint8_t>(cv::Point(x, y)) = 255;
					}
				}
			}*/
			int32_t x0 = (estimate_pose_.x() / 0.025 + 0.5 * 1024) + 0.5f;
			int32_t y0 = (estimate_pose_.y() / 0.025 + 0.5 * 1024) + 0.5f;
			float sin_th = sin(estimate_pose_.theta());
			float cos_th = cos(estimate_pose_.theta());
			//cv::arrowedLine(img,cv::Point(x0,y0),cv::Point(x0+cos_th*16.0f,y0+sin_th*16.0f),cv::Scalar(0, 0, 0),2,8,0,0.5);
			//cv::circle(img, cv::Point(x0, y0), 5, cv::Scalar(0, 0, 0));
			cv::imshow("img", img);
			//cv::imshow("img2", img2);
			//cv::imshow("img4", img4);
			cv::waitKey(10);
			rate.sleep();
		}
	}

private:
	bool poseDifferentEnough(const Beta &beta)
	{
		if (map_is_empty_)
		{
			map_is_empty_ = false;
			return true;
		}
		else
		{
			float distance = std::sqrt(
					std::pow(beta.x(), 2) + std::pow(beta.y(), 2));
			float angle = std::abs(beta.theta());
			if (distance > 0.5)
			{
				return true;
			}
			else if (angle > 0.05)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	void scanCallback(const sensor_msgs::LaserScanConstPtr &scan)
	{
		//std::cout << "counter_(" << counter_++ << ")" << std::endl;
		projector_.projectLaser(*scan, points_container_);
		Beta new_pose = matcher_.match(maps_container_, points_container_,
				estimate_pose_);
		std::cout << "scanCallback:" << new_pose.transpose() << std::endl;
		//
		Eigen::Matrix3f trans = Transformer::toMatrix(new_pose);
		if (poseDifferentEnough(last_draw_pose_ - estimate_pose_))
		{
			for (std::vector<naive_mapping::GridMap>::iterator frontmap =
					maps_container_.begin(); frontmap != maps_container_.end();
					frontmap++)
			{
				(*frontmap).insert(trans, points_container_);
			}
			last_draw_pose_ = new_pose;
		}
		estimate_pose_ = new_pose;
		//
		//tf_pose_ = Transformer::toTf(estimate_pose);
		return;
	}

private:
	ros::Subscriber scan_subscriber_;
	ros::Publisher map_publisher_;
	tf::Transform tf_pose_;
	Beta estimate_pose_, last_draw_pose_;
	bool map_is_empty_;
private:
	naive_mapping::Geometry projector_;
	naive_mapping::Matcher matcher_;
	std::vector<GridMap> maps_container_;
	std::vector<Point> points_container_;
private:
	boost::thread* map_publish_thread_;

};

//#include<fstream>
//#include <iostream>
//using namespace std;

int main(int argc, char **argv)
{
	//ofstream of("/home/bosskwei/Desktop/naive_out.txt");
	     // 获取文件out.txt流缓冲区指针
	      //streambuf* fileBuf = of.rdbuf();

	     // 设置cout流缓冲区指针为out.txt的流缓冲区指针
	      //cout.rdbuf(fileBuf);
	//GridMap map;
	//map.setOccupied(Index(512,512)); map.setOccupied(Index(513,512));
	//cout << map.getBilinearInterpolation(Point(0.01,0)).transpose() << endl;
	Node node;
	node.init(argc, argv);
	return 0;
}
