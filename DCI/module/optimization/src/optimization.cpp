/*****************************************************************//**
 * \file   optimization.cpp
 * \brief
 *
 * \author Fred
 * \date   December 2024
 *********************************************************************/
#include "optimization.h"

void Optimization::optimize()
{
	//logger->info("Start optimization...");
	std::map<Component::Ptr, int>component_varIdx;
	std::set<Component::Ptr> constraintedComponents;

	typedef Variable<double> Variable;
	typedef LinearExpression<double> Objective;
	typedef LinearProgram<double> LinearProgram;
	typedef LinearProgram::Constraint Constraint;
	LinearProgramSolver solver;
	Objective obj;
	//1. add obj
	int totalVarIdx = 0;
	int varIdx = 0;
	const auto& toBasicFinalSize = energyTerm_->getBasicFinalSize();
	for (const auto& comp : energyTerm_->getPset()->components()) {
		double coverage = comp->attri().energyterm.coverage;
		double compact = comp->attri().energyterm.compact;
		double persistence = comp->attri().energyterm.persistence;
		obj.add_coefficient(varIdx, lamda_.lamda_coverage * coverage * -1.0);
		obj.add_coefficient(varIdx, lamda_.lamda_compact * compact * -1.0);
		obj.add_coefficient(varIdx, lamda_.lamda_persistence * persistence * -1.0);

		//add the toBasicFinal energy term
		for (const auto& level : comp->attri().energyterm.toBasicFinal) {
			if (level.first == this->processLevel_) {
				//TODO annotation persDifference for now
				//obj.add_coefficient(varIdx, -1.0 * (level.second.persDifference));
				//double supportPointsPercent = static_cast<double> (level.second.supportPoints) / toBasicFinalSize.at(level.first);
				//obj.add_coefficient(varIdx, -1.0 * (supportPointsPercent));
				obj.add_coefficient(varIdx, -1.0 * (comp->attri().energyterm.stability[level.first][0]));// use basic firstly try
			}
		}
		component_varIdx[comp] = varIdx;
		varIdx++;
	}
	totalVarIdx += varIdx;

	//2.add constraint
	int constraintCompNum = 0;
	const auto basicLevelComponentFinal_thisLevel = energyTerm_->getBasicLevelComponentFinal()[this->processLevel_];
	if (basicLevelComponentFinal_thisLevel.size() < 1) {
		//logger->error("No basic components in this level!");
		exit(1);
	}
	Constraint constraint;
	int constraintIdx = 0;
	for (const auto& comp : energyTerm_->getPset()->components()) {
		if (std::find(basicLevelComponentFinal_thisLevel.begin(), basicLevelComponentFinal_thisLevel.end(), comp) != basicLevelComponentFinal_thisLevel.end()) {
			constraint.add_coefficient(component_varIdx[comp], 1.0);
			constraintedComponents.insert(comp);
			constraintIdx++;
		}
	}
	//constraint.set_bounds(Constraint::LOWER, 0.8 * constraintIdx, 0.0);
	constraint.set_bounds(Constraint::UPPER, 0.0, 0.2 * basicLevelComponentFinal_thisLevel.size());
#if 0
	for (const auto& levelComp : energyTerm_->getBasicLevelComponentFinal()) {//fro each level
		Constraint constraint;
		int constraintIdx = 0;
		if (levelComp.first == this->processLevel_) {
			for (const auto& comp : levelComp.second) {//for each component of level
				if (comp->attri().levelIndices[levelComp.first].size() > componentThreshold_) {
					constraint.add_coefficient(component_varIdx[comp], 1.0);
					constraintedComponents.insert(comp);
					constraintIdx++;

					//double pointsLevelCoverage = 1.0 - (comp->attri().levelIndices[levelComp.first].size() / static_cast<double>(comp->size()));

					//double pointsLevelCoverage = comp->attri().energyConstraintTerm[levelComp.first];
					//obj.add_coefficient(component_varIdx[comp], pointsLevelCoverage);
				}
			}
			//constraint.add_coefficient(constraintIdx, -0.8 * levelComp.second.size());
			constraint.set_bounds(Constraint::LOWER, 0.8 * constraintIdx, 0.0);
			program_.add_constraint(constraint);
			constraintCompNum = constraintIdx;
		}
		else {
			for (const auto& comp : levelComp.second) {
				//if (comp->attri().levelIndices[levelComp.first].size() < componentThreshold_) {
				//	continue;
				//}
				//double pointsLevelCoverage = 1.0;
				//obj.add_coefficient(component_varIdx[comp], pointsLevelCoverage);
			}
		}
	}
#endif
	//logger->info("Total number of constraints: {}", constraintIdx);

	//3.solver
	program_.set_objective(obj, LinearProgram::MINIMIZE);
	for (std::size_t i = 0; i < energyTerm_->getPset()->components().size(); i++) {
		program_.add_variable(Variable(Variable::BINARY));
	}
	// if (solver.solve(&program_, LinearProgramSolver::SolverName::GUROBI)) {
	// 	//logger->info("Optimization done!");
	// }
	// else {
	// 	//logger->error("Optimization failed!");
	// }
	//4.result process
	std::set<Component::Ptr> selectedComponents;
	std::set<Component::Ptr> selectedConstraintComps, unselectedConstraintComps;
	const std::vector<double>& X = solver.get_result();
	for (const auto& comp : energyTerm_->getPset()->components()) {
		if (static_cast<int>(std::round(X[component_varIdx[comp]])) == 1) {
			selectedComponents.insert(comp);
			if (constraintedComponents.find(comp) != constraintedComponents.end()) {
				selectedConstraintComps.insert(comp);
			}
		}
		else {
			if (constraintedComponents.find(comp) != constraintedComponents.end()) {
				unselectedConstraintComps.insert(comp);
			}
		}
	}
#if 0
	//save the constraint components to corrsponding folder as ply files
		//selected

	for (const auto& comp : selectedConstraintComps) {
		std::string fileName = "/" + std::string("selectedConstraintComps") + "/" + std::to_string(component_varIdx[comp]) + ".ply";
		std::set<Component::Ptr> compSet = { comp };
		energyTerm_->getPset()->saveComponents2Cloud(compSet, fileName);
	}
	//unselected
	for (const auto& comp : unselectedConstraintComps) {
		std::string fileName = "/" + std::string("unselectedConstraintComps") + "/" + std::to_string(component_varIdx[comp]) + ".ply";
		std::set<Component::Ptr> compSet = { comp };
		energyTerm_->getPset()->saveComponents2Cloud(compSet, fileName);
	}
#endif
	if (energyTerm_->getPset()->saveComponents2Cloud(selectedComponents, "finalResult" + this->processLevel_ + ".ply")) {
		//logger->info("Optimized successful and save result done!");
	}
	else {
		//logger->error("Save selected components to local failed!");
	}
}

void Optimization::optimize1()
{
}

void Optimization::saveCompWithStable()
{
	std::map<std::string, std::set<Component::Ptr>> levelComponent;//save the components of each level statified by stability
	for (const auto& comp : energyTerm_->getPset()->components()) {
		for (const auto& level : comp->attri().energyterm.stability) {
			if (level.second[1] <= 0.008 && comp->indices().size() > 100) {
				levelComponent[level.first].insert(comp);
			}
		}
	}

	for (const auto& level : levelComponent) {
		std::string fileName = "/compWithStable_" + level.first + ".ply";
		energyTerm_->getPset()->saveComponents2Cloud(level.second, fileName);
#if 0
		for (const auto& comp : level.second) {
			energyTerm_->getPset()->saveIndices2Cloud(comp->indices(), "/compWithStable_" + level.first + "_comp" + std::to_string(comp->index()) + ".ply");
		}
#endif
	}
}