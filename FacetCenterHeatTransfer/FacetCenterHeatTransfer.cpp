#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>

FacetCenterHeatTransfer::FacetCenterHeatTransfer(const GridData& gridData)
	: grid2D(gridData)
{
	this->initializeLinearSystem();
	this->initializeTemperatureVectors();
	return;
}

void FacetCenterHeatTransfer::initializeLinearSystem(void)
{
	const unsigned numberOfElements = this->grid2D.elements.size();
	this->linearSystem.matrix = Eigen::MatrixXd::Zero(numberOfElements,numberOfElements);
	this->linearSystem.independent = Eigen::VectorXd::Zero(numberOfElements);
	return;
}

void FacetCenterHeatTransfer::initializeTemperatureVectors(void)
{
	this->temperature = Eigen::VectorXd::Zero(this->grid2D.elements.size());
	return;
}