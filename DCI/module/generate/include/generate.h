/*****************************************************************//**
 * \file   generate.h
 * \brief  generate the final result of multiscale primitive detection by the similarity and stability (persisitence)
 *
 * \author Fred
 * \date   April 2025
 *********************************************************************/
#ifndef GENERATE_H
#define GENERATE_H
#include "../../../src/globInfo.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/ml.hpp>
using namespace std;
using namespace cv;
using namespace cv::ml;
using namespace Eigen;

class GenerateCluster {
public:
	GenerateCluster(int outterCluster_ = 3, int innerCluster_ = 3) : outterClusterNum(outterCluster_), innerClusterNum(innerCluster_) {}

public:
	int outterClusterNum = 3;
	int innerClusterNum = 3;
	vector<double> ReadValues(const string& path);
	vector<int> ReorderLabels(const Mat& centers, vector<int>& labels, bool reverse = true);
	void FirstClustering(const vector<double>& data, int n_clusters, vector<int>& out_labels, Mat& out_centers);
	vector<vector<int>> SecondClustering(const vector<double>& list2, vector<int>& labels1, int n_clusters);
	std::vector<std::vector<int>> getCluster(const vector<double>& simliValue, const vector<double>& stabValue, const int firstLevelNum = 3, const int SecondLevelNum = 3);
};

#endif
