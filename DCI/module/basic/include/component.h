/*****************************************************************//**
 * \file   component.h
 * \brief  class "component" has the attribute of vector, save the point index in pointset.points belong to each component
 *
 * \author Fred
 * \date   December 2024
 *********************************************************************/
#pragma once
#ifndef _COMPONENT_H_
#define _COMPONENT_H_

 //#include "./point_set.h"
#include "./basic_types.h"
#include <vector>
#include <memory>

struct toBasicFinalEnergy {
	double persDifference;
	int supportPoints;
};

struct componentEnergyTerm {
	double coverage;
	double compact;
	double persistence;
	std::map<std::string, toBasicFinalEnergy> toBasicFinal;
	std::map<std::string, std::array<double, 2>> stability;//array[0] for basicStabilityEnergy, array[1] for basicFinalStabilityEnergy
};

struct componentAttri {
	int birth;
	int death;
	int persistence;
	Plane_3 plane;// fit the plane of component points as ax+by+cz+d=0 => {A,B,C,D}
	std::array<double, 3> compStab;
	double compStab_cura;
	componentEnergyTerm energyterm;
	std::map<std::string, std::vector<int>> levelIndices;
	std::map<std::string, double> energyConstraintTerm;
};

class PointSet;
class Component :public std::vector<unsigned int> {
public:
	typedef std::shared_ptr<Component> Ptr;
private:
	int componentIndex;
	PointSet* point_set_;
	componentAttri attri_;
public:
	Component(PointSet* pset_ = 0) :point_set_(pset_) {};
	~Component() {
		clear();
	};
	void set_point_set(PointSet* pset) { point_set_ = pset; };
	void set_Attri(int birth, int death) {
		attri_.birth = birth;
		attri_.death = death;
		attri_.persistence = death - birth;
	}
	void setPlane(Plane_3 plane) {
		attri_.plane = plane;
	}
	void setEnergyTerm(double cov, double comp, double pers) {
		attri_.energyterm.coverage = cov;
		attri_.energyterm.compact = comp;
		attri_.energyterm.persistence = pers;
	}
	void setEnergyTerm(double cov, double comp, double pers, std::map<std::string, toBasicFinalEnergy> toBasicFinal) {
		attri_.energyterm.coverage = cov;
		attri_.energyterm.compact = comp;
		attri_.energyterm.persistence = pers;
		setToBasicFinal(toBasicFinal);
	}
	void setLevelIndices(const std::map<std::string, std::vector<int>> levelIndices) {
		attri_.levelIndices = levelIndices;
	}
	void setEnergyConstraintTerm(const std::map<std::string, double> energyConstraintTerm) {
		attri_.energyConstraintTerm = energyConstraintTerm;
	}
	void setToBasicFinal(const std::map<std::string, toBasicFinalEnergy> toBasicFinal_) {
		attri_.energyterm.toBasicFinal = toBasicFinal_;
	}
	void setToBasicFinal(const std::string level, toBasicFinalEnergy basicFinalEnergy) {
		attri_.energyterm.toBasicFinal[level] = basicFinalEnergy;
	}
	void setIndex(const int index) {
		componentIndex = index;
	}
	void setStability(std::array<double, 3> stability) {
		attri_.compStab = stability;
	}
	void setStability(double compStab) {
		attri_.compStab_cura = compStab;
	}
	void setStabilityEnergy(const std::string level, std::array<double, 2> stability) {
		attri_.energyterm.stability[level] = stability;
	}
	void setStability(const std::map<std::string, std::array<double, 2>> stability) {
		attri_.energyterm.stability = stability;
	}
	inline PointSet* point_set() { return point_set_; };
	inline componentAttri attri() { return attri_; };
	inline componentEnergyTerm energyterm() { return attri_.energyterm; };
	std::vector<int> indices() {
		std::vector<int> indices;
		for (auto i : *this) {
			indices.push_back(i);
		}
		return indices;
	}
	int index() {
		return componentIndex;
	}
	std::array<double, 3> stability() {
		return attri_.compStab;
	}
	const double stability(bool isCura) {
		return attri_.compStab_cura;
	}
public:
	Plane_3 fitPlane(const Component::Ptr comp);
};
#endif // !_COMPONENT_H_
