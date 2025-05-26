#pragma once
#ifndef INITIALCORRESPONDENCES_H_
#define INITIALCORRESPONDENCES_H_
#include <iostream>
#include <vector>
//PCL
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/pcl_search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
namespace ICS
{
	//typedef pcl::PointXYZ Pointxyz;
	//typedef pcl::PointCloud<Pointxyz> Pointcloud;
	//typedef pcl::PointCloud<Pointxyz>::Ptr PointcloudPtr;
	//typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHcloudPtr;

	void Voxeldownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double resolution);

	void ExtractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointIndicesPtr ISS_Idx, double resolution);

	void FPFHcomputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointIndicesPtr Index, pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH,double resolution);

	void CorrespondencesSearching(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_S, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_T, int k,std::vector<std::pair<int,int>> &corr);

	void GetInitialCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT_in,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS_corr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT_corr, double resolution);

}
#endif // !INITIALCORRESPONDENCES_H_

