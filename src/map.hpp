#pragma once
//#include <ros/ros.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "naive_mapping/types.h"

namespace naive_mapping
{

class Cell
{
public:
	Cell() :
			P_(0)
	{
	}
	bool isOccupied() const
	{
		return P_ > 0;
	}

	bool isFree() const
	{
		return P_ < 0;
	}
	float getProbability() const
	{
		//return exp(P_) / (exp(P_) + 1.0f);
		//return P_;
		//return P_ / 100.0f;
		return 100.0f / (1.0f + std::exp(-P_ / 32.0f));
		//return 1.0f / (1.0f + std::exp(-P_ / 32.0f));
	}
	void updateSetOccupied()
	{
		//if (P_ < 50.0f)
		//	P_ += 0.405465;
		if (P_ < 100)
		{
			P_ += 10;
		}
	}
	void updateSetFree()
	{
		//if (P_ > -50.0f)
		//	P_ -= 0.405465;
		if (P_ > -100)
		{
			P_ -= 1;
		}
	}

private:
	int8_t P_;
	//int8_t Flag_;
};

class GridMap
{
public:
	GridMap(std::string frame = "map") :
			resolution_(0.025), width_(1024), height_(1024)
	{
		map_ = std::vector<Cell>(width_ * height_);
		frame_ = frame;
	}
	GridMap(std::string frame, const GridMap &upper) :
			resolution_(upper.resolution_ * 2), width_(upper.width_ / 2), height_(
					upper.height_ / 2)
	{
		map_ = std::vector<Cell>(width_ * height_);
		frame_ = frame;
	}
	Eigen::Vector3f getBilinearInterpolation(const Point &point) const
	{
		float x = point.x() / resolution_ + 0.5 * width_;
		float y = point.y() / resolution_ + 0.5 * height_;

		int32_t x1 = std::floor(x);
		int32_t y1 = std::floor(y);
		int32_t x2 = x1 + 1; //std::ceil(x);
		int32_t y2 = y1 + 1; //std::ceil(y);
		//
		float Q11 = getCell(Index(x1, y1)).getProbability();
		float Q21 = getCell(Index(x2, y1)).getProbability();
		float Q12 = getCell(Index(x1, y2)).getProbability();
		float Q22 = getCell(Index(x2, y2)).getProbability();
		//std::cout << "(" << x1 << "," << y1 << "):" << (int)Q11 << " (" << x2 << "," << y1 << "):" << (int)Q21 << std::endl;
		//std::cout << "(" << x1 << "," << y2 << "):" << (int)Q12 << " (" << x2 << "," << y2 << "):" << (int)Q22 << std::endl;
		//
		float R1 = ((x2 - x) * Q11 + (x - x1) * Q21);
		float R2 = ((x2 - x) * Q12 + (x - x1) * Q22);
		float P = ((y2 - y) * R1 + (y - y1) * R2);
		//assert(P<=100);
		//P = P / 100.0f;
		//P = sigmod(P);
		//
		float Dx = (y - y1) * (Q22 - Q12) + (y2 - y) * (Q21 - Q11);
		float Dy = (x - x1) * (Q22 - Q21) + (x2 - x) * (Q12 - Q11);
		//
		return Eigen::Vector3f(P, Dx, Dy);
	}
	void insert(const Eigen::Matrix3f pose, const std::vector<Point> &points)
	{
		int32_t x0 = (pose(0, 2) / resolution_ + 0.5 * width_) + 0.5f;
		int32_t y0 = (pose(1, 2) / resolution_ + 0.5 * height_) + 0.5f;
		//
		int32_t x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		bool first = 1;
		for (std::vector<Point>::const_iterator current = points.begin();
				current != points.end(); current++)
		{
			Eigen::Vector3f point = pose * (*current);

			x2 = (point.x() / resolution_ + 0.5 * width_) + 0.5f;
			y2 = (point.y() / resolution_ + 0.5 * height_) + 0.5f;
			if (norm(x1, y1, x2, y2) < 10)
			{
				lineBresenham(x1, y1, x2, y2, 1);
			}
			lineBresenham(x0, y0, x2, y2, 0);
			x1 = x2;
			y1 = y2;
		}
	}
	inline Cell getCell(const Index index) const
	{
		//
		if (!(index.x() >= 0 && index.x() < width_)
				|| !(index.y() >= 0 && index.y() < height_))
		{
			//std::cout << "out off map" << std::endl;
			return empty_cell_;
		}
		//assert(index.x() >= 0 && index.x() < width_);
		//assert(index.y() >= 0 && index.y() < height_);
		//
		size_t n = index.y() * width_ + index.x();
		return map_[n];
	}
	inline Cell &getCell(const Index index)
	{
		//
		//assert(index.x() >= 0 && index.x() < width_);
		//assert(index.y() >= 0 && index.y() < height_);
		if (!(index.x() >= 0 && index.x() < width_)
				|| !(index.y() >= 0 && index.y() < height_))
		{
			//std::cout << "out off map" << std::endl;
			return empty_cell_;
		}
		//
		size_t n = index.y() * width_ + index.x();
		return map_[n];
	}
	void setOccupied(const Index &index)
	{
		getCell(index).updateSetOccupied();
	}
	void setFree(const Index &index)
	{
		getCell(index).updateSetFree();
	}
	std::string getFrame() const
	{
		return frame_;
	}
private:
	float sigmod(float x) const
	{
		return 1.0f / (1.0f + std::exp(-x / 32.0f));
	}
	/*void setOccupied(const Point &point)
	 {
	 int32_t x = (point.x() / resolution_ + 0.5 * width_) + 0.5;
	 int32_t y = (point.y() / resolution_ + 0.5 * height_) + 0.5;
	 //
	 setOccupied(Index(x, y));
	 }*/
	void lineBresenham(int x1, int y1, int x2, int y2, bool occupancy)
	{
		int dx = x2 - x1;
		int dy = y2 - y1;
		int ux = ((dx > 0) << 1) - 1; //x的增量方向，取或-1
		int uy = ((dy > 0) << 1) - 1; //y的增量方向，取或-1
		int x = x1, y = y1, eps; //eps为累加误差

		eps = 0;
		dx = abs(dx);
		dy = abs(dy);
		if (dx > dy)
		{
			for (x = x1; x != x2 + ux; x += ux)
			{
				if (occupancy)
					setOccupied(Index(x, y));
				else
					setFree(Index(x, y));
				eps += dy;
				if ((eps << 1) >= dx)
				{
					y += uy;
					eps -= dx;
				}
			}
		}
		else
		{
			for (y = y1; y != y2 + uy; y += uy)
			{
				if (occupancy)
					setOccupied(Index(x, y));
				else
					setFree(Index(x, y));
				eps += dx;
				if ((eps << 1) >= dy)
				{
					x += ux;
					eps -= dy;
				}
			}
		}
	}
	inline float norm(int32_t x0, int32_t y0, int32_t x1, int32_t y1) const
	{
		return std::sqrt(std::pow((x1 - x0), 2) + std::pow((y1 - y0), 2));
	}
private:
	std::string frame_;
	std::vector<Cell> map_;
	Cell empty_cell_;
	float resolution_;
	size_t width_, height_;
};

}
