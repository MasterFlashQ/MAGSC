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


#include <iostream>
#include <chrono>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "InitialCorrespondences.h"
#include "MAGSC.h"

int main(int argc,char** argv)
{
	//INPUT:
	// 1. path to the source point cloud
	std::string fnameS = argv[1];
	// 2. path to the target point cloud
	std::string fnameT = argv[2];
	// 3. resolution threshold (default 0.1)
	double resolution = atof(argv[3]);

	pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloudS(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloudT(new pcl::PointCloud<pcl::PointXYZ>);

	//support pcd or ply
	if (fnameS.substr(fnameS.find_last_of('.') + 1) == "pcd") {
		pcl::io::loadPCDFile(fnameS, *origin_cloudS);
		pcl::io::loadPCDFile(fnameT, *origin_cloudT);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "ply") {
		pcl::io::loadPLYFile(fnameS, *origin_cloudS);
		pcl::io::loadPLYFile(fnameT, *origin_cloudT);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr corrS(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr corrT(new pcl::PointCloud<pcl::PointXYZ>);

	ICS::GetInitialCorrespondences(origin_cloudS, origin_cloudT, corrS, corrT, resolution);
	std::cout << "corrS size:" << corrS->size() << " " << "corrT size:" << corrT->size() << std::endl;

	//--------------MAGSC-------------------------------------------
	auto sta = std::chrono::system_clock::now();
	MAGSC::MagscReg<pcl::PointXYZ> Magsc;
	Magsc.setResolution(resolution);
	Magsc.setSourcepcd(corrS);
	Magsc.setTargetpcd(corrT);
	Magsc.align();
	auto end = std::chrono::system_clock::now();
	std::cout << "/*MAGSC time cost:" << double(std::chrono::duration_cast<std::chrono::milliseconds>(end - sta).count()) / 1000.0 << std::endl;
	std::cout << "/*=================================================*/" << std::endl;

	std::cout << Magsc.getTransformation() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*origin_cloudS, *aligned, Magsc.getTransformation());

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("brefore registration"));
	viewer->setBackgroundColor(255, 255, 255);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> reg_t(origin_cloudT, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(origin_cloudT, reg_t, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> reg_s(origin_cloudS, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(origin_cloudS, reg_s, "source cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("after registration"));
	viewer2->setBackgroundColor(255, 255, 255);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> reg_t2(origin_cloudT, 255, 0, 0);
	viewer2->addPointCloud<pcl::PointXYZ>(origin_cloudT, reg_t2, "target cloud");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> reg_s2(aligned, 0, 0, 255);
	viewer2->addPointCloud<pcl::PointXYZ>(aligned, reg_s2, "source cloud");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		viewer2->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
}
