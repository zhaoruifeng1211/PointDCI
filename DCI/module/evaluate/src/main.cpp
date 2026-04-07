#include <iostream>
#include "evaluate.h"
int main(int argc, char* argv[]) {
	if (argc < 3) {
		std::cout << "Usage: " << argv[0] << " <.vg> <alphaValue>" << std::endl;
		exit(-1);
	}
	std::string vgfilePath = argv[1];
	std::ifstream input(vgfilePath);
	if (!input.is_open()) {
		throw std::runtime_error("Error: cannot open file " + vgfilePath);
		exit(-1);
	}
	double alphaValue = std::stod(argv[2]);
	PointSet* pset = new PointSet();
	pset->readVG(vgfilePath);
	std::cout << "read vg done! read " << pset->groups().size() << " groups" << std::endl;
	evaluate* evalu = new evaluate();
	evalu->baseFileName_ = pset->baseFilePath();
	evalu->process(*pset, alphaValue);
	std::cout << "compute RMSE" << std::endl;
	evalu->RMSE(*pset);
}