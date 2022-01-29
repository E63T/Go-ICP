/********************************************************************
Header File for Go-ICP Class
Last modified: Apr 21, 2014

"Go-ICP: Solving 3D Registration Efficiently and Globally Optimally"
Jiaolong Yang, Hongdong Li, Yunde Jia
International Conference on Computer Vision (ICCV), 2013

Copyright (C) 2013 Jiaolong Yang (BIT and ANU)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#pragma once

#include <queue>

#include "jly_3ddt.h"

#define PI 3.1415926536
#define SQRT3 1.732050808 
// TODO : Remove constants or move it to .cpp file

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>

namespace goicp
{



typedef struct _ROTNODE
{
	float a, b, c, w;
	float ub, lb;
	int l;
	friend bool operator < (const struct _ROTNODE & n1, const struct _ROTNODE & n2)
	{
		if(n1.lb != n2.lb)
			return n1.lb > n2.lb;
		else
			return n1.w < n2.w;
			//return n1.ub > n2.ub;
	}
	
}ROTNODE;

typedef struct _TRANSNODE
{
	float x, y, z, w;
	float ub, lb;
	friend bool operator < (const struct _TRANSNODE & n1, const struct _TRANSNODE & n2)
	{
		if(n1.lb != n2.lb)
			return n1.lb > n2.lb;
		else
			return n1.w < n2.w;
			//return n1.ub > n2.ub;
	}
}TRANSNODE;

/********************************************************/



/********************************************************/

constexpr static int MAXROTLEVEL = 20;

template<typename Point>
class GoICP
{
public:
	using PointCloudPtr = typename pcl::PointCloud<Point>::Ptr;

	PointCloudPtr pModel, pData, pTransformed;

	ROTNODE initNodeRot;
	TRANSNODE initNodeTrans;

	DT3D<Point> dt;

	int maxIcpIter;

	ROTNODE optNodeRot;
	TRANSNODE optNodeTrans;

	GoICP();
	float Register();
	void BuildDT();

	float MSEThresh;
	float SSEThresh;
	float icpThresh;

	float optError;
	Eigen::Matrix3d optR;
	Eigen::Vector3d optT;

	clock_t clockBegin;

	float trimFraction;
	int inlierNum;
	bool doTrim;

private:
	//temp variables
	std::vector<float> normData;
	std::vector<float> minDis;
	std::vector<std::vector<float>> maxRotDis;
	float * maxRotDisL;
	
	pcl::IterativeClosestPoint<Point, Point, float> icp;

	float ICP(Eigen::Matrix3d& R_icp, Eigen::Vector3d& t_icp);
	float InnerBnB(float* maxRotDisL, TRANSNODE* nodeTransOut);
	float OuterBnB();
	void Initialize();
};

/********************************************************/

} // namespace goicp