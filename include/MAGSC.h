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
//Thanks to the work of Zhou, et al:
//https://github.com/isl-org/FastGlobalRegistration
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

		//*@brief Set the resolution of filtered point cloud
		//*@param[in] float resolution Input resolution 
		//*@return void
		void setResolution(float resolution)
		{
			resolution_ = resolution;
		}

		//*@brief Set the multiple of angle when check the symmetry, the default is 4
		//*@param[in] int delta_times
		//*@return void
		void setDeltaTimes(int delta_times)
		{
			delta_times_ = delta_times;
		}

		//*@brief Set the source point cloud
		//*@param[in] PointcloudPtr source_pcd
		//*@return void
		void setSourcepcd(PointcloudPtr source_pcd)
		{
			source_pcd_ = source_pcd;
		}

		//*@brief Set the target point cloud
		//*@param[in] PointcloudPtr target_pcd
		//*@return void
		void setTargetpcd(PointcloudPtr target_pcd)
		{
			target_pcd_ = target_pcd;
		}

		//*@brief Start the registration procedure
		//*@return void
		void align();

		//*@brief Get the optimal transformation computed by MAGSC
		//*@return Eigen::Matrix4f
		Eigen::Matrix4f getTransformation()
		{
			return Transformation_final;
		}

		//*@brief Get the transformation computed by inliers extracted by MAGSC
		//*@return Eigen::Matrix4f
		Eigen::Matrix4f getTransformationSVD()
		{
			return Transformation_SVD;
		}

	private:
		//*@brief Compute adjacency matrix by distance consistency
		//*@return void
		void computeCompatibility();

		//*@brief Generate the candidate of the maximum consensus
		//*@param[out] std::vector<std::set<int>> &Cluster_candidate_ The candidate set of the maximum consensus
		//*@param[out] std::vector<std::pair<int, std::pair<int, int>>> &Cluster_candidate_size_ The size of each candidate set
		//*@param[out] std::vector<std::vector<std::pair<int, int>>> &Correlation_degree_ The Correlation degree of every node in each candidate set
		//*@return void
		void Graphcluster(std::vector<std::set<int>> &Cluster_candidate_, 
			std::vector<std::pair<int, std::pair<int, int>>> &Cluster_candidate_size_,
			std::vector<std::vector<std::pair<int, int>>> &Correlation_degree_);

		//*@brief Search the maximum consensus set using spatial consistency
		//*@param[in] std::vector<std::set<int>> &Cluster_candidate_ The candidate set of the maximum consensus
		//*@param[in] std::vector<std::pair<int, std::pair<int, int>>> &Cluster_candidate_size_ The size of each candidate set
		//*@param[in] std::vector<std::vector<std::pair<int, int>>> &Correlation_degree_ The Correlation degree of every node in each candidate set
		//*@param[out] std::vector<int>& maximum_consensus_index_ The index of the inliers
		//*@return void
		void GloballySpatialConsistency(std::vector<std::set<int>>& Cluster_candidate_,
			std::vector<std::pair<int, std::pair<int, int>>>& Cluster_candidate_size_,
			std::vector<std::vector<std::pair<int, int>>>& Correlation_degree_,
			std::vector<int>& maximum_consensus_index_);

		//*@brief Compute the distance from a point to a line
		//*@param[in] Point& P_p The point
		//*@param[in] Point& P_a The endpoint of a straight line
		//*@param[in] Point& P_b The endpoint of a straight line
		//*@return float The distance from P_p to the line constructed by P_a and P_b
		float point2lineDistance(const Point& P_p, const Point& P_a, const Point& P_b);

		//*@brief Check whether the two points are at the same side of the plane
		//*@param[in] int p1_index_ The index of one point that constitutes the plane
		//*@param[in] int p2_index_ The index of one point that constitutes the plane
		//*@param[in] int p3_index_ The index of one point that constitutes the plane
		//*@param[in] int p4_index_ The inspected point
		//*@return bool True indicates two points are at the same side of the plane,otherwise not
		bool Symmetrycheck(const int p1_index_, const int p2_index_, const int p3_index_, const int p4_index_);

		//*@brief Optimize the transformation computed by inliers
		//*@return void
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

	};
}

#include "MAGSC.hpp"
