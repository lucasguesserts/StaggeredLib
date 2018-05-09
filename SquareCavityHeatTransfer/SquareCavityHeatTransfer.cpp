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
		this->linearSystem.matrix(elementIndex,elementIndex) += rho * cp * elementVolume;
		this->linearSystem.independent[elementIndex] += rho * cp * elementVolume * this->oldTemperature[elementIndex];
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

void SquareCavityHeatTransfer::addDiffusiveTerm(void)
{
	for(auto& staggeredQuadrangle: this->grid2D.staggeredQuadrangles)
		addDiffusiveTerm(*staggeredQuadrangle);
	return;
}

void SquareCavityHeatTransfer::addDiffusiveTerm(StaggeredElement2D& staggeredQuadrangle)
{
	const unsigned frontElementIndex = staggeredQuadrangle.elements[0]->getIndex();
	const unsigned backElementIndex = staggeredQuadrangle.elements[1]->getIndex();
	ScalarStencil diffusiveTerm = this->computeDiffusiveTerm(staggeredQuadrangle);
	// matrix
	ScalarStencil matrixDiffusiveTerm = this->timeImplicitCoefficient * diffusiveTerm;
	this->linearSystem.addScalarStencil(backElementIndex, (-1)*matrixDiffusiveTerm);
	this->linearSystem.addScalarStencil(frontElementIndex, matrixDiffusiveTerm);
	// independent
	double independentDiffusiveTerm = (1-this->timeImplicitCoefficient) * diffusiveTerm * this->oldTemperature;
	this->linearSystem.independent[backElementIndex] += independentDiffusiveTerm;
	this->linearSystem.independent[frontElementIndex] += - independentDiffusiveTerm;
	return;
}

ScalarStencil SquareCavityHeatTransfer::computeDiffusiveTerm(StaggeredElement2D& staggeredQuadrangle)
{
	Eigen::Vector3d areaVector = staggeredQuadrangle.getAreaVector();
	VectorStencil gradient = this->grid2D.computeVectorStencilOnQuadrangle(staggeredQuadrangle,this->scalarStencilOnVertices);
	return areaVector * this->k * this->timeInterval * gradient;
}

void SquareCavityHeatTransfer::applyBoundaryConditions(void)
{
	for(auto& dirichlet: this->dirichletBoundaries)
		this->applyBoundaryCondition(dirichlet);
	return;
}

void SquareCavityHeatTransfer::applyBoundaryCondition(DirichletBoundaryCondition& dirichlet)
{
	for(unsigned i=0 ; i<dirichlet.staggeredTriangle.size() ; ++i)
		this->applyDirichletBoundaryConditionInStaggeredTriangle(*(dirichlet.staggeredTriangle[i]), dirichlet.prescribedValue[i]);
}

void SquareCavityHeatTransfer::applyDirichletBoundaryConditionInStaggeredTriangle(StaggeredElement2D& staggeredTriangle, const double prescribedValue)
{
	// TODO: simplify this function
	const unsigned elementIndex = staggeredTriangle.elements[0]->getIndex();
	Eigen::Vector3d gradientVector = staggeredTriangle.getCentroid() - staggeredTriangle.elements[0]->getCentroid();
	gradientVector = gradientVector / gradientVector.squaredNorm();
	double coeff = - staggeredTriangle.getAreaVector().dot(gradientVector) * this->k * this->timeInterval;
	this->linearSystem.matrix(elementIndex,elementIndex) += coeff * this->timeImplicitCoefficient;
	this->linearSystem.independent(elementIndex) -= coeff * (1 - this->timeImplicitCoefficient) * this->oldTemperature[elementIndex];
	this->linearSystem.independent(elementIndex) += coeff * prescribedValue;
	return;
}

Eigen::VectorXd SquareCavityHeatTransfer::nextTimeStep(void)
{
	const unsigned numberOfElements = this->linearSystem.independent.size();
	this->linearSystem.matrix = Eigen::MatrixXd::Zero(numberOfElements,numberOfElements);
	this->linearSystem.independent = Eigen::VectorXd::Zero(numberOfElements);
	this->addAccumulationTerm();
	this->addDiffusiveTerm();
	this->applyBoundaryConditions();
	return this->linearSystem.solve();
}