#include "InitialCorrespondences.h"

void ICS::Voxeldownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double resolution)
{
	pcl::VoxelGrid<pcl::PointXYZ> vox;
	vox.setLeafSize(resolution, resolution, resolution);
	vox.setInputCloud(cloud_in);
	vox.filter(*cloud_out);
}

void ICS::ExtractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointIndicesPtr ISS_Idx, double resolution)
{
	double iss_salient_radius_ = 6 * resolution;
	double iss_non_max_radius_ = 4 * resolution;
	double iss_gamma_21_(0.975);
	double iss_gamma_32_(0.975);
	double iss_min_neighbors_(4);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
	iss.setSearchMethod(tree);
	iss.setInputCloud(cloud_in);
	iss.setSalientRadius(iss_salient_radius_);
	iss.setNonMaxRadius(iss_non_max_radius_);
	iss.setThreshold21(iss_gamma_21_);
	iss.setThreshold32(iss_gamma_32_);
	iss.setMinNeighbors(iss_min_neighbors_);
	iss.setNumberOfThreads(1);
	iss.compute(*keypoints);
	ISS_Idx->indices = iss.getKeypointsIndices()->indices;
	ISS_Idx->header = iss.getKeypointsIndices()->header;
}

void ICS::FPFHcomputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointIndicesPtr Index, pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_out, double resolution)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nor_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	nor_est.setInputCloud(cloud_in);
	nor_est.setSearchMethod(tree);
	nor_est.setRadiusSearch(3 * resolution);
	nor_est.compute(*normals);

	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> FPFH_est;
	FPFH_est.setInputCloud(cloud_in);
	FPFH_est.setInputNormals(normals);
	FPFH_est.setSearchMethod(tree);
	FPFH_est.setRadiusSearch(8 * resolution);
	FPFH_est.setIndices(Index);
	FPFH_est.compute(*FPFH_out);
}

void ICS::CorrespondencesSearching(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_S, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_T, int k, std::vector<std::pair<int, int>>& corr)
{

	pcl::KdTreeFLANN<pcl::FPFHSignature33> treeS;
	treeS.setInputCloud(FPFH_S);
	pcl::KdTreeFLANN<pcl::FPFHSignature33> treeT;
	treeT.setInputCloud(FPFH_T);

	for (int i = 0; i < FPFH_S->size(); i++)
	{
		std::vector<int> idx;
		std::vector<float> dis;
		int k_near = treeT.nearestKSearch(FPFH_S->points[i], k, idx, dis);
		for (int j = 0; j < idx.size(); j++)
		{
			bool flag = false;
			std::vector<int> idx_temp;
			std::vector<float> dis_temp;
			int k_temp = treeS.nearestKSearch(FPFH_T->points[idx[j]], k, idx_temp, dis_temp);
			for (int l = 0; l < idx_temp.size(); l++)
			{
				if (idx_temp[l] == i)
				{
					flag = true;
					break;
				}
			}
			if (flag)
			{
				std::pair<int, int> match;
				match.first = i;
				match.second = idx[j];
				corr.push_back(match);
			}
		}
	}
}

void ICS::GetInitialCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS_corr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT_corr, double resolution)
{
	int k = 5;

	pcl::PointCloud<pcl::PointXYZ>::Ptr sampleS(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampleT(new pcl::PointCloud<pcl::PointXYZ>);
	ICS::Voxeldownsample(cloudS_in, sampleS,resolution);
	ICS::Voxeldownsample(cloudT_in, sampleT, resolution);
	std::cout << "Sample finish" << std::endl;
	std::cout << "SampleS size:" << sampleS->size() << " " << "SampleT size:" << sampleT->size() << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsS(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsT(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr keyidxS(new pcl::PointIndices);
	pcl::PointIndicesPtr keyidxT(new pcl::PointIndices);
	ICS::ExtractKeypoints(sampleS, keypointsS, keyidxS, resolution);
	ICS::ExtractKeypoints(sampleT, keypointsT, keyidxT, resolution);
	std::cout << "Extract keypoints finish" << std::endl;

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHS(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHT(new pcl::PointCloud<pcl::FPFHSignature33>);
	ICS::FPFHcomputation(sampleS, keyidxS, FPFHS, resolution);
	ICS::FPFHcomputation(sampleT, keyidxT, FPFHT, resolution);
	std::cout << "FPFH computation finish" << std::endl;

	std::vector<std::pair<int, int>> corr;
	ICS::CorrespondencesSearching(FPFHS, FPFHT,k, corr);
	std::cout << "Search correspondences finish" << std::endl;

	for (auto pair_i : corr)
	{
		int S_index = pair_i.first;
		int T_index = pair_i.second;

		cloudS_corr->push_back(keypointsS->points[S_index]);
		cloudT_corr->push_back(keypointsT->points[T_index]);
	}
}