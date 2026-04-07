/*****************************************************************//**
 * \file   optimization.h
 * \brief
 *
 * \author Fred
 * \date   December 2024
 *********************************************************************/
#ifndef _OPTIMIZATION_H
#define _OPTIMIZATION_H

#include "linear_program.h"
#include "linear_program_solver.h"
#include "../../energyTerm/include/energyTerm.h"

struct Lamda {
	double lamda_level_0;
	double lamda_level_1;
	double lamda_level_2;
	double lamda_coverage;
	double lamda_compact;
	double lamda_persistence;
};

class energyTerm;
class PointSet;
class Optimization {
public:
	typedef std::shared_ptr<Component> Ptr;
public:
	Optimization(energyTerm* energyTerm_) :energyTerm_(energyTerm_)/*, logger(LoggerFactory<Optimization>::getLogger())*/ {};
	void optimize();
	void optimize1();
	void setLamda(double coverage, double compact, double persistence, double level0, double level1, double level2) {
		lamda_.lamda_coverage = coverage;
		lamda_.lamda_compact = compact;
		lamda_.lamda_persistence = persistence;
		lamda_.lamda_level_0 = level0;
		lamda_.lamda_level_1 = level1;
		lamda_.lamda_level_2 = level2;
	}
	void setComponentThreshold(int threshold) {
		componentThreshold_ = threshold;
	}
	void setBasePath(const std::string& basePath) {
		basePath_ = basePath;
	}
	void setProcessLevel(const std::string processLevel) {
		processLevel_ = processLevel;
	}

	/** tools for pre-check */

	void saveCompWithStable();

	const std::string getPorcessLevel() {
		return processLevel_;
	}

private:
	energyTerm* energyTerm_;
	LinearProgram<double> program_;
	Lamda lamda_;
	int componentThreshold_ = 50;
	std::string basePath_;
	std::string processLevel_ = "0";

	//std::shared_ptr<spdlog::logger> logger;
};

#endif // !_OPTIMIZATION_H
