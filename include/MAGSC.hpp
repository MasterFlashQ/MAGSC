#pragma once
#include "MAGSC.h"

template<typename Point>
inline float MAGSC::MagscReg<Point>::point2lineDistance(const Point& P_p, const Point& P_a, const Point& P_b)
{
	Eigen::Vector3f P(P_p.x, P_p.y, P_p.z);
	Eigen::Vector3f A(P_a.x, P_a.y, P_a.z);
	Eigen::Vector3f B(P_b.x, P_b.y, P_b.z);
	Eigen::Vector3f AP = P - A;
	Eigen::Vector3f AB = B - A;
	Eigen::Vector3f cross = AP.cross(AB);
	float crossLength = cross.norm();
	float ABLength = AB.norm();
	return crossLength / ABLength;
}

template<typename Point>
inline bool MAGSC::MagscReg<Point>::Symmetrycheck(const int p1_index_, const int p2_index_, const int p3_index_, const int p4_index_)
{
	Eigen::Vector3d _1_3_S;
	Eigen::Vector3d _2_3_S;
	Eigen::Vector3d _1_3_T;
	Eigen::Vector3d _2_3_T;

	_1_3_S << source_pcd_->points[p1_index_].x - source_pcd_->points[p3_index_].x,
		source_pcd_->points[p1_index_].y - source_pcd_->points[p3_index_].y,
		source_pcd_->points[p1_index_].z - source_pcd_->points[p3_index_].z;
	_2_3_S << source_pcd_->points[p2_index_].x - source_pcd_->points[p3_index_].x,
		source_pcd_->points[p2_index_].y - source_pcd_->points[p3_index_].y,
		source_pcd_->points[p2_index_].z - source_pcd_->points[p3_index_].z;
	_1_3_T << target_pcd_->points[p1_index_].x - target_pcd_->points[p3_index_].x,
		target_pcd_->points[p1_index_].y - target_pcd_->points[p3_index_].y,
		target_pcd_->points[p1_index_].z - target_pcd_->points[p3_index_].z;
	_2_3_T << target_pcd_->points[p2_index_].x - target_pcd_->points[p3_index_].x,
		target_pcd_->points[p2_index_].y - target_pcd_->points[p3_index_].y,
		target_pcd_->points[p2_index_].z - target_pcd_->points[p3_index_].z;

	Eigen::Vector3d _1_3_2_S = _1_3_S.cross(_2_3_S);
	Eigen::Vector3d _1_3_2_T = _1_3_T.cross(_2_3_T);

	Eigen::Vector3d _4_3_S;
	Eigen::Vector3d _4_3_T;
	_4_3_S << source_pcd_->points[p4_index_].x - source_pcd_->points[p3_index_].x,
		source_pcd_->points[p4_index_].y - source_pcd_->points[p3_index_].y,
		source_pcd_->points[p4_index_].z - source_pcd_->points[p3_index_].z;
	_4_3_T << target_pcd_->points[p4_index_].x - target_pcd_->points[p3_index_].x,
		target_pcd_->points[p4_index_].y - target_pcd_->points[p3_index_].y,
		target_pcd_->points[p4_index_].z - target_pcd_->points[p3_index_].z;
	float angle_threshold, _4_3_angle_S, _4_3_angle_T;
	float dis_3_4 = pcl::geometry::distance(target_pcd_->points[p4_index_], target_pcd_->points[p3_index_]);
	angle_threshold = 180 * asin(resolution_ / dis_3_4) / M_PI;
	_4_3_angle_S = 180 * acos(_1_3_2_S.dot(_4_3_S) / (_1_3_2_S.norm() * _4_3_S.norm())) / M_PI;
	_4_3_angle_T = 180 * acos(_1_3_2_T.dot(_4_3_T) / (_1_3_2_T.norm() * _4_3_T.norm())) / M_PI;

	if (abs(_4_3_angle_S - _4_3_angle_T) < delta_times_ * angle_threshold)
		return true;
	else
		return false;
}

template<typename Point>
inline void MAGSC::MagscReg<Point>::computeCompatibility()
{
	int N = source_pcd_->size();
	compatibility_matrix.setZero(N, N);
	compatibility_degree.resize(N, std::pair<int,int>(0,0));
	compatibility_set.resize(N);
	degree_.resize(N, 0);
#pragma omp parallel for
	for (int i = 0; i < N; i++)
	{
		compatibility_degree[i].second = i;
		int index_i = i;
		for (int j = 0; j < N; j++)
		{
			if (j >= i)
				break;
			int index_j = j;

			Point pt_iS = source_pcd_->points[i];
			Point pt_jS = source_pcd_->points[j];
			Point pt_iT = target_pcd_->points[i];
			Point pt_jT = target_pcd_->points[j];

			float dis_i_j_S = pcl::geometry::distance(pt_iS, pt_jS);
			float dis_i_j_T = pcl::geometry::distance(pt_iT, pt_jT);

			if (abs(dis_i_j_S - dis_i_j_T) < 2 * resolution_)
			{
#pragma omp critical
				{
					compatibility_matrix(i, j) = 1;
					compatibility_matrix(j, i) = 1;
					compatibility_degree[i].first++;
					compatibility_degree[j].first++;
					compatibility_set[i].insert(j);
					compatibility_set[j].insert(i);
					degree_[i] += 1;
					degree_[j] += 1;
				}
			}
		}
	}
}

template<typename Point>
inline void MAGSC::MagscReg<Point>::Graphcluster(std::vector<std::set<int>>& Cluster_candidate_,
	std::vector<std::pair<int, std::pair<int, int>>>& Cluster_candidate_size_,
	std::vector<std::vector<std::pair<int, int>>>& Correlation_degree_)
{
	std::set<int> Seed_points_index;//seed points index for candidate generate
	for (int i = 0; i < compatibility_degree.size(); i++)
	{
		Seed_points_index.insert(compatibility_degree[i].second);//set all nodes as seed points
	}
	for (int index_i = 0; index_i < compatibility_degree.size(); index_i++)
	{
		int i = compatibility_degree[index_i].second;//index of nodes
		if (compatibility_degree[index_i].first < 2)//node degree<2,break
		{
			break;
		}
		if (Seed_points_index.count(i) != 1)
		{
			continue;
		}
		std::vector<std::pair<int, int>> CD_of_i;
		std::set<int> graph_cluster_set_of_i;

		for (int index_j = 0; index_j < compatibility_degree.size(); index_j++)
		{
			int j = compatibility_degree[index_j].second;
			if (compatibility_degree[index_j].first < 2)
			{
				break;
			}
			if (index_i == index_j)
			{
				continue;
			}
			if (compatibility_matrix(i, j) == 1)
			{
				std::set<int> intersect_set_i_j;
				set_intersection(compatibility_set[i].begin(), compatibility_set[i].end(), compatibility_set[j].begin(), compatibility_set[j].end(), inserter(intersect_set_i_j, intersect_set_i_j.begin()));
				if (intersect_set_i_j.size() > 0)
				{
					Seed_points_index.erase(j);
					set_union(intersect_set_i_j.begin(), intersect_set_i_j.end(), graph_cluster_set_of_i.begin(), graph_cluster_set_of_i.end(), inserter(graph_cluster_set_of_i, graph_cluster_set_of_i.begin()));
					for (auto k = intersect_set_i_j.begin(); k != intersect_set_i_j.end(); k++)
					{
						Seed_points_index.erase(*k);
					}
					CD_of_i.push_back(std::make_pair(intersect_set_i_j.size(), j));
				}
			}
		}
		if (graph_cluster_set_of_i.size() > 0)
		{
			Cluster_candidate_.push_back(graph_cluster_set_of_i);
			Cluster_candidate_size_.push_back(std::make_pair(graph_cluster_set_of_i.size(), std::make_pair(i, Cluster_candidate_.size() - 1)));
			Correlation_degree_.push_back(CD_of_i);
		}
	}

}

template<typename Point>
inline void MAGSC::MagscReg<Point>::GloballySpatialConsistency(std::vector<std::set<int>>& Cluster_candidate_,
	std::vector<std::pair<int, std::pair<int, int>>>& Cluster_candidate_size_,
	std::vector<std::vector<std::pair<int, int>>>& Correlation_degree_,
	std::vector<int> & maximum_consensus_index_)
{
	int maximum_count = 0;
	for (auto index_i : Cluster_candidate_size_)
	{
		if (index_i.first < maximum_count)
			break;
		int p1_index = index_i.second.first;
		if (degree_[p1_index]< maximum_count)
			continue;
		int cluster_index = index_i.second.second;
		std::vector<std::pair<int, int>> correlation_degree_set = Correlation_degree_[cluster_index];
		std::sort(correlation_degree_set.rbegin(), correlation_degree_set.rend());
		std::set<int> cluster_i = Cluster_candidate_[cluster_index];
		for (auto index_j : correlation_degree_set)
		{
			int p2_index = index_j.second;
			if (degree_[p2_index] < maximum_count)
				continue;
			if (index_j.first < maximum_count - 1)
				break;

			float disS_1_2;
			disS_1_2 = pcl::geometry::distance(source_pcd_->points[p1_index], source_pcd_->points[p2_index]);
			float disT_1_2;
			disT_1_2 = pcl::geometry::distance(target_pcd_->points[p1_index], target_pcd_->points[p2_index]);
			if (disS_1_2 < 2 * resolution_ || disT_1_2 < 2 * resolution_)
				continue;

			std::vector<int> maximal_consensus;
			std::vector<int> inliers_temp;
			for (auto index_k : cluster_i)
			{
				int p3_index = index_k;
				if (compatibility_matrix(p2_index, p3_index) != 1 || degree_[p3_index] <maximum_count)
					continue;
				float p3Top12S;
				p3Top12S = point2lineDistance(source_pcd_->points[p3_index], source_pcd_->points[p2_index], source_pcd_->points[p1_index]);
				float p3Top12T;
				p3Top12T = point2lineDistance(target_pcd_->points[p3_index], target_pcd_->points[p2_index], target_pcd_->points[p1_index]);
				if (p3Top12T < 2 * resolution_ || p3Top12S < 2 * resolution_)
					continue;
				inliers_temp.clear();
				for (auto index_l : cluster_i)
				{
					int p4_index = index_l;
					if (compatibility_matrix(p2_index, p4_index) == 1 && compatibility_matrix(p4_index, p3_index) == 1)
					{
						if (Symmetrycheck(p1_index, p2_index, p3_index, p4_index))
							inliers_temp.push_back(p4_index);
					}
				}
				inliers_temp.push_back(p3_index);
			}
			if (inliers_temp.size() > maximal_consensus.size())
				maximal_consensus = inliers_temp;
			maximal_consensus.push_back(p2_index);
			maximal_consensus.push_back(p1_index);
			if (maximal_consensus.size() > maximum_count)
			{
				maximum_consensus_index_=maximal_consensus;
				maximum_count = maximum_consensus_index_.size();
			}
		}
	}
}

template<typename Point>
inline Eigen::Matrix4f MAGSC::MagscReg<Point>::GNCoptimization()
{
	Eigen::Matrix4f trans;
	trans = Transformation_SVD;
	int N = source_pcd_->size();
	float div_factor_ = 1.4;
	double max_corr_dist_ = resolution_;
	std::vector<double> s(N, 1.0);
	pcl::transformPointCloud(*source_pcd_, *source_pcd_, trans);
	float niu;

	float alpha[15] = { 1,0.5,0.25,0,-0.25,-0.5,-1,-2,-4,-8,-16,-32,-64,-128,-256 };
	for (int i = 0; i < 15; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			niu = pow(div_factor_, 3 - j) * max_corr_dist_;
			// niu=max_corr_dist_;
			const int nvariable = 6;	// 3 for rotation and 3 for translation
			Eigen::MatrixXd JTJ(nvariable, nvariable);
			Eigen::MatrixXd JTr(nvariable, 1);
			Eigen::MatrixXd J(nvariable, 1);
			JTJ.setZero();
			JTr.setZero();

			double r;
			double r2 = 0.0;
			for (int c = 0; c < source_pcd_->size(); c++)
			{
				Eigen::Vector3f p, q;

				float dis_S_T = pcl::geometry::distance(source_pcd_->points[c], target_pcd_->points[c]);
				if (dis_S_T > 10 * max_corr_dist_)
				{
					continue;
				}
				q << source_pcd_->points[c].x, source_pcd_->points[c].y, source_pcd_->points[c].z;
				p << target_pcd_->points[c].x, target_pcd_->points[c].y, target_pcd_->points[c].z;

				Eigen::Vector3f rpq = p - q;

				int c2 = c;
				float e2 = rpq.dot(rpq);

				if (alpha[i] == 0)
				{
					float alpha_i = alpha[i];
					s[c2] = 1 / (2 * pow(niu, 2) + e2);
				}
				else
				{
					float alpha_i = alpha[i];
					s[c2] = pow(1 + (e2 / (niu * niu)) / abs(alpha_i - 2), alpha_i / 2 - 1);
				}

				J.setZero();
				J(1) = -q(2);
				J(2) = q(1);
				J(3) = -1;
				r = rpq(0);
				JTJ += J * J.transpose() * s[c2];
				JTr += J * r * s[c2];

				J.setZero();
				J(2) = -q(0);
				J(0) = q(2);
				J(4) = -1;
				r = rpq(1);
				JTJ += J * J.transpose() * s[c2];
				JTr += J * r * s[c2];

				J.setZero();
				J(0) = -q(1);
				J(1) = q(0);
				J(5) = -1;
				r = rpq(2);
				JTJ += J * J.transpose() * s[c2];
				JTr += J * r * s[c2];
			}

			Eigen::MatrixXd result(nvariable, 1);
			result = -JTJ.llt().solve(JTr);

			Eigen::Affine3d aff_mat;
			aff_mat.linear() = (Eigen::Matrix3d)Eigen::AngleAxisd(result(2), Eigen::Vector3d::UnitZ())
				* Eigen::AngleAxisd(result(1), Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(result(0), Eigen::Vector3d::UnitX());
			aff_mat.translation() = Eigen::Vector3d(result(3), result(4), result(5));

			Eigen::Matrix4f delta = aff_mat.matrix().cast<float>();

			trans = delta * trans;
			pcl::transformPointCloud(*source_pcd_, *source_pcd_, delta);
		}
	}
	return trans;
}

template<typename Point>
inline void MAGSC::MagscReg<Point>::align()
{
	computeCompatibility();
	std::cout << "/*-----finish computeCompatibility-----*/" << std::endl;
	std::sort(compatibility_degree.rbegin(), compatibility_degree.rend());

	std::vector<std::set<int>> Cluster_candidate;//cluster candidate
	std::vector<std::pair<int, std::pair<int, int>>> Cluster_candidate_size;//cluster candidate size for sort
	//std::vector<int> search_index;
	std::vector<std::vector<std::pair<int, int>>> Correlation_degree;//correaltion degree
	Graphcluster(Cluster_candidate, Cluster_candidate_size, Correlation_degree);
	std::cout << "/*------finish Graphcluster-----*/" << std::endl;
	std::sort(Cluster_candidate_size.rbegin(), Cluster_candidate_size.rend());

	std::vector<int> maximum_consensus_index;
	GloballySpatialConsistency(Cluster_candidate, Cluster_candidate_size, Correlation_degree,maximum_consensus_index);
	std::cout << "/*-----finish GloballySpatialConsistency-----*/" << std::endl;

	std::cout << "inliers count:" << maximum_consensus_index.size() <<std::endl;

	PointcloudPtr inlierSpcd(new Pointcloud);
	PointcloudPtr inlierTpcd(new Pointcloud);

	for (auto i : maximum_consensus_index)
	{
		inlierSpcd->push_back(source_pcd_->points[i]);
		inlierTpcd->push_back(target_pcd_->points[i]);
	}

	pcl::io::savePLYFile(S_path_, *inlierSpcd);
	pcl::io::savePLYFile(T_path_, *inlierTpcd);

	pcl::registration::TransformationEstimationSVD<Point, Point> SVD_estimator;
	SVD_estimator.estimateRigidTransformation(*inlierSpcd, *inlierTpcd, Transformation_SVD);
	Transformation_final = GNCoptimization();
}

