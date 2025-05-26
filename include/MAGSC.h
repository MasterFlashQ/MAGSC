#pragma once
/**=====================================================================================================
* Copyright 2024, THE COLLEGE OF GEOSCIENCE AND SURVEYING ENGINEERING, CHINA UNIVERSITY OF MINING AND TECHNOLOGY (BEIJING)
* BEIJING, CHINA
* All Rights Reserved
* Authors: Fanqiang Meng.
* Do not hesitate to contact the authors if you have any question or find any bugs
* Email: fq.meng@student.cumtb.edu.cn
* See LICENSE for the license information
//=======================================================================================================
*/
* Thanks to the work of Zhou, et al:
* https://github.com/isl-org/FastGlobalRegistration
*/

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/impl/io.hpp>
#include <pcl/io/ply_io.h>
#include <string>
#include <set>

namespace MAGSC
{
	template<typename Point>
	class MagscReg
	{
	public:
		typedef typename pcl::PointCloud<Point> Pointcloud;
		typedef typename pcl::PointCloud<Point>::Ptr PointcloudPtr;
		MagscReg()
		{
		}
		void setResolution(float resolution)
		{
			resolution_ = resolution;
		}
		void setDeltaTimes(int delta_times)
		{
			delta_times_ = delta_times;
		}
		//void setDelta(int delta)
		//{
		//	delta_ = delta;
		//}
		void setSourcepcd(PointcloudPtr source_pcd)
		{
			source_pcd_ = source_pcd;
		}
		void setTargetpcd(PointcloudPtr target_pcd)
		{
			target_pcd_ = target_pcd;
		}
		void align();
		Eigen::Matrix4f getTransformation()
		{
			return Transformation_final;
		}
		Eigen::Matrix4f getTransformationSVD()
		{
			return Transformation_SVD;
		}

		void setMSpath(std::string S_path, std::string T_path)
		{
			S_path_ = S_path;
			T_path_ = T_path;
		}

	private:
		void computeCompatibility();
		void Graphcluster(std::vector<std::set<int>> &Cluster_candidate_, 
			std::vector<std::pair<int, std::pair<int, int>>> &Cluster_candidate_size_,
			std::vector<std::vector<std::pair<int, int>>> &Correlation_degree_);
		void GloballySpatialConsistency(std::vector<std::set<int>>& Cluster_candidate_,
			std::vector<std::pair<int, std::pair<int, int>>>& Cluster_candidate_size_,
			std::vector<std::vector<std::pair<int, int>>>& Correlation_degree_,
			std::vector<int>& maximum_consensus_index_);
		float point2lineDistance(const Point& P_p, const Point& P_a, const Point& P_b);
		bool Symmetrycheck(const int p1_index_, const int p2_index_, const int p3_index_, const int p4_index_);
		Eigen::Matrix4f GNCoptimization();
		float resolution_;
		int delta_times_=4;
		PointcloudPtr source_pcd_, target_pcd_;
		std::vector<std::pair<int, int>> compatibility_degree;
		std::vector<std::set<int>> compatibility_set;
		Eigen::MatrixXi compatibility_matrix;
		std::vector<int> degree_;
		Eigen::Matrix4f Transformation_SVD = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f Transformation_final = Eigen::Matrix4f::Identity();

		std::string S_path_, T_path_;
	};
}

#include "MAGSC.hpp"
