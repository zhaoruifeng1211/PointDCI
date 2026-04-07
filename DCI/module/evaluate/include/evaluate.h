/*****************************************************************//**
 * \file   evaluate.h
 * \brief
 *
 * \author Fred
 * \date   May 2025
 *********************************************************************/
#pragma once
#ifndef EVALUATE_H
#define EVALUATE_H
#include<string>

#include"point_set.h"
class evaluate {
public:
	evaluate()/* :logger(LoggerFactory<evaluate>::getLogger())*/ {
	};
	~evaluate() {
	}
public:
	std::string baseFileName_;
public:
	void process(PointSet& pset, double alphaValue);
	bool RMSE(PointSet& pset);

	/** IO */
	bool saveResult(const std::string& filename, const std::ostringstream& buffer);
};
#endif
