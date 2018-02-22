#define _USE_MATH_DEFINES
#include <cmath>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>

SquareCavityHeatTransfer::SquareCavityHeatTransfer(const GridData& gridData)
	: grid2D(gridData)
{
	const unsigned numberOfElements = this->grid2D.elements.size();
	this->linearSystem.matrix = Eigen::MatrixXd::Zero(numberOfElements,numberOfElements);
	this->linearSystem.independent = Eigen::VectorXd::Zero(numberOfElements);
	this->oldTemperature = Eigen::VectorXd::Zero(numberOfElements);
	this->temperature = Eigen::VectorXd::Zero(numberOfElements);
	return;
}

void SquareCavityHeatTransfer::addAccumulationTerm(void)
{
	for(Element* element: this->grid2D.elements)
	{
		const unsigned elementIndex = element->getIndex();
		this->linearSystem.matrix(elementIndex,elementIndex) = rho * cp * element->getVolume();
		this->linearSystem.independent[elementIndex] = rho * cp * element->getVolume() * this->oldTemperature[elementIndex];
	}
	return;
}

Eigen::VectorXd SquareCavityHeatTransfer::computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3> coordinates)
{
	const unsigned numberOfPoints = coordinates.rows();
	double x, y;
	Eigen::VectorXd solution;
	solution.resize(numberOfPoints);
	for(unsigned positionIndex=0 ; positionIndex<numberOfPoints ; ++positionIndex)
	{
		x = coordinates(positionIndex,0);
		y = coordinates(positionIndex,1);
		solution[positionIndex] = std::sin(M_PI*x) * std::sinh(M_PI*y) / std::sinh(M_PI);
	}
	return solution;
}
