#define _USE_MATH_DEFINES
#include <cmath>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>

SquareCavityHeatTransfer::SquareCavityHeatTransfer(const GridData& gridData)
	: grid2D(gridData)
{
	this->initializeLinearSystem();
	this->initializeTemperatureVectors();
	this->initializeScalarStencilOnVertices();
	return;
}

void SquareCavityHeatTransfer::initializeLinearSystem(void)
{
	const unsigned numberOfElements = this->grid2D.elements.size();
	this->linearSystem.matrix = Eigen::MatrixXd::Zero(numberOfElements,numberOfElements);
	this->linearSystem.independent = Eigen::VectorXd::Zero(numberOfElements);
	return;
}

void SquareCavityHeatTransfer::initializeTemperatureVectors(void)
{
	const unsigned numberOfElements = this->grid2D.elements.size();
	this->oldTemperature = Eigen::VectorXd::Zero(numberOfElements);
	this->temperature = Eigen::VectorXd::Zero(numberOfElements);
	return;
}

void SquareCavityHeatTransfer::initializeScalarStencilOnVertices(void)
{
	this->scalarStencilOnVertices = this->grid2D.computeScalarStencilOnVertices();
}

void SquareCavityHeatTransfer::addAccumulationTerm(void)
{
	for(Element* element: this->grid2D.elements)
	{
		const unsigned elementIndex = element->getIndex();
		const double elementVolume = element->getVolume();
		this->linearSystem.matrix(elementIndex,elementIndex) = rho * cp * elementVolume;
		this->linearSystem.independent[elementIndex] = rho * cp * elementVolume * this->oldTemperature[elementIndex];
	}
	return;
}

Eigen::VectorXd SquareCavityHeatTransfer::computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3> coordinates)
{
	const unsigned numberOfPoints = coordinates.rows();
	double x, y;
	Eigen::VectorXd solution = Eigen::VectorXd::Zero(numberOfPoints);
	for(unsigned positionIndex=0 ; positionIndex<numberOfPoints ; ++positionIndex)
	{
		x = coordinates(positionIndex,0);
		y = coordinates(positionIndex,1);
		solution[positionIndex] = std::sin(M_PI*x) * std::sinh(M_PI*y) / std::sinh(M_PI);
	}
	return solution;
}

void SquareCavityHeatTransfer::addDiffusiveTerm(StaggeredQuadrangle& staggeredQuadrangle)
{
	Eigen::Vector3d areaVector = staggeredQuadrangle.getAreaVector();
	VectorStencil gradient = this->grid2D.computeVectorStencilOnQuadrangle(staggeredQuadrangle,this->scalarStencilOnVertices);
	ScalarStencil scalarStencil = areaVector * k * timeInterval * timeImplicitCoefficient * gradient;
	this->linearSystem.addScalarStencil(staggeredQuadrangle.elements[0]->getIndex(), (-1)*scalarStencil);
	this->linearSystem.addScalarStencil(staggeredQuadrangle.elements[1]->getIndex(), scalarStencil);
	return;
}
