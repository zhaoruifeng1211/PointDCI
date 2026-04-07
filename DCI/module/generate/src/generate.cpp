#include "../include/generate.h"

vector<double> GenerateCluster::ReadValues(const string& path)
{
	vector<double> data;
	ifstream ifs(path);
	double v;
	while (ifs >> v) {
		data.push_back(v);
	}
	return data;
}

vector<int> GenerateCluster::ReorderLabels(const Mat& centers, vector<int>& labels, bool reverse)
{
	int k = centers.rows;
	vector<pair<double, int>> center_idx;
	for (int i = 0; i < k; ++i) {
		center_idx.emplace_back(centers.at<float>(i, 0), i);
	}
	if (reverse) sort(center_idx.begin(), center_idx.end(), greater<>());
	else sort(center_idx.begin(), center_idx.end());

	unordered_map<int, int> map_old_new;
	for (int new_i = 0; new_i < k; ++new_i) {
		map_old_new[center_idx[new_i].second] = new_i;
	}

	vector<int> reordered(labels.size());
	for (size_t i = 0; i < labels.size(); ++i)
		reordered[i] = map_old_new[labels[i]];
	return reordered;
}

void GenerateCluster::FirstClustering(const vector<double>& data, int n_clusters, vector<int>& out_labels, Mat& out_centers)
{
	Mat samples(data.size(), 1, CV_32F);
	for (size_t i = 0; i < data.size(); ++i)
		samples.at<float>(i, 0) = static_cast<float>(data[i]);

	Mat labels;
	kmeans(samples, n_clusters, labels,
		TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 10000, 0.0001),
		3, KMEANS_PP_CENTERS, out_centers);

	out_labels.resize(data.size());
	for (size_t i = 0; i < data.size(); ++i)
		out_labels[i] = labels.at<int>(i, 0);
}

vector<vector<int>> GenerateCluster::SecondClustering(const vector<double>& list2, vector<int>& labels1, int n_clusters)
{
	int N = list2.size();
	vector<vector<int>> all_indices; // 每个最终簇的索引集合

	int max_label = *max_element(labels1.begin(), labels1.end());
	for (int label = 0; label <= max_label; ++label) {
		vector<int> idx;
		for (int i = 0; i < N; ++i)
			if (labels1[i] == label) idx.push_back(i);

		if (idx.size() < n_clusters) {
			all_indices.push_back(idx); // 直接作为一个簇
			continue;
		}

		Mat samples(idx.size(), 1, CV_32F);
		for (size_t i = 0; i < idx.size(); ++i) {
			samples.at<float>(i, 0) = static_cast<float>(list2[idx[i]]);
		}

		Mat labels;
		Mat centers;
		kmeans(samples, n_clusters, labels,
			TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 10000, 0.0001),
			3, KMEANS_PP_CENTERS, centers);

		vector<int> label_vec(labels.rows);
		for (int i = 0; i < labels.rows; ++i)
			label_vec[i] = labels.at<int>(i, 0);

		vector<int> reordered = ReorderLabels(centers, label_vec, true);

		vector<vector<int>> cluster_indices(n_clusters);
		for (size_t i = 0; i < idx.size(); ++i) {
			int c = labels.at<int>(i, 0);
			cluster_indices[c].push_back(idx[i]);
		}
		for (auto& cluster : cluster_indices) {
			all_indices.push_back(cluster);
		}
	}
	return all_indices;
}

std::vector<std::vector<int>>GenerateCluster::getCluster(const vector<double>& simliValue, const vector<double>& stabValue, const int firstLevelNum, const int SecondLevelNum)
{
	//1st clustering
	std::vector<int> labels1;
	Mat centers1;
	FirstClustering(simliValue, firstLevelNum, labels1, centers1);
	labels1 = ReorderLabels(centers1, labels1, true);

	//2nd clustering
	auto second = SecondClustering(stabValue, labels1, SecondLevelNum);

	return second;
}