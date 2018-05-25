#define _USE_MATH_DEFINES
#include <cmath>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>

SquareCavityHeatTransfer::SquareCavityHeatTransfer(const std::string& fileName)
	: grid2D(fileName)
{
	this->initializeLinearSystem();
	this->initializeTemperatureVectors();
	this->initializeScalarStencilOnVertices();
	this->initializeDiffusiveTerm();
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

void SquareCavityHeatTransfer::initializeDiffusiveTerm(void)
{
	this->diffusiveTerm.resize(this->grid2D.staggeredElements.size());
	for(auto staggeredQuadrangle: this->grid2D.staggeredQuadrangles)
		this->diffusiveTerm[staggeredQuadrangle->getIndex()] = this->computeDiffusiveTerm(*staggeredQuadrangle);
	return;
}

void SquareCavityHeatTransfer::insertDirichletBoundaryCondition(const std::string& boundaryName, const std::function<double(Eigen::Vector3d)> prescribedValueFunction)
{
	DirichletBoundaryCondition dirichlet;
	dirichlet.staggeredTriangle = this->grid2D.boundary[boundaryName].staggeredTriangle;
	dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
	for(unsigned count=0 ; count<dirichlet.staggeredTriangle.size() ; ++count)
		dirichlet.prescribedValue[count] = prescribedValueFunction(dirichlet.staggeredTriangle[count]->getCentroid());
	this->dirichletBoundaries.emplace_back(std::move(dirichlet));
	return;
}

void SquareCavityHeatTransfer::insertDirichletBoundaryCondition(const std::string& boundaryName, const double prescribedValue)
{
	auto dirichletFunction = [prescribedValue](Eigen::Vector3d) -> double { return prescribedValue; };
	this->insertDirichletBoundaryCondition(boundaryName, dirichletFunction);
	return;
}

void SquareCavityHeatTransfer::addAccumulationTerm(void)
{
	this->addAccumulationTermToMatrix();
	this->addAccumulationTermToIndependent();
	return;
}

void SquareCavityHeatTransfer::addAccumulationTermToMatrix(void)
{
	for(Element* element: this->grid2D.elements)
	{
		const unsigned elementIndex = element->getIndex();
		const double elementVolume = element->getVolume();
		this->linearSystem.matrix(elementIndex,elementIndex) += rho * cp * elementVolume;
	}
	return;
}

void SquareCavityHeatTransfer::addAccumulationTermToIndependent(void)
{
	for(Element* element: this->grid2D.elements)
	{
		const unsigned elementIndex = element->getIndex();
		const double elementVolume = element->getVolume();
		this->linearSystem.independent[elementIndex] += rho * cp * elementVolume * this->oldTemperature[elementIndex];
	}
	return;
}

Eigen::VectorXd SquareCavityHeatTransfer::computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3>& coordinates)
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

Eigen::VectorXd SquareCavityHeatTransfer::computeAnalyticalSolution(void)
{
	Eigen::Matrix<double,Eigen::Dynamic,3> elementsCentroid;
	elementsCentroid.resize(this->grid2D.elements.size(), Eigen::NoChange);
	for(auto element: this->grid2D.elements)
	{
		Eigen::Vector3d centroid = element->getCentroid();
		elementsCentroid(element->getIndex(),0) = centroid(0);
		elementsCentroid(element->getIndex(),1) = centroid(1);
		elementsCentroid(element->getIndex(),2) = centroid(2);
	}
	return SquareCavityHeatTransfer::computeAnalyticalSolution(elementsCentroid);
}

void SquareCavityHeatTransfer::addDiffusiveTerm(void)
{
	this->addDiffusiveTermToMatrix();
	this->addDiffusiveTermToIndependent();
	return;
}

void SquareCavityHeatTransfer::addDiffusiveTerm(StaggeredElement2D& staggeredQuadrangle)
{
	const unsigned frontElementIndex = staggeredQuadrangle.elements[0]->getIndex();
	const unsigned backElementIndex = staggeredQuadrangle.elements[1]->getIndex();
	ScalarStencil diffusiveTerm = (this->k * this->timeInterval) * this->diffusiveTerm[staggeredQuadrangle.getIndex()];
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

void SquareCavityHeatTransfer::addDiffusiveTermToMatrix(void)
{
	for(auto& staggeredQuadrangle: this->grid2D.staggeredQuadrangles)
	{
		const unsigned frontElementIndex = staggeredQuadrangle->elements[0]->getIndex();
		const unsigned backElementIndex = staggeredQuadrangle->elements[1]->getIndex();
		ScalarStencil matrixDiffusiveTerm = (this->timeImplicitCoefficient * this->k * this->timeInterval) * this->diffusiveTerm[staggeredQuadrangle->getIndex()];
		this->linearSystem.addScalarStencil(backElementIndex, (-1)*matrixDiffusiveTerm);
		this->linearSystem.addScalarStencil(frontElementIndex, matrixDiffusiveTerm);
	}
	return;
}

void SquareCavityHeatTransfer::addDiffusiveTermToIndependent(void)
{
	for(auto& staggeredQuadrangle: this->grid2D.staggeredQuadrangles)
	{
		const unsigned frontElementIndex = staggeredQuadrangle->elements[0]->getIndex();
		const unsigned backElementIndex = staggeredQuadrangle->elements[1]->getIndex();
		double independentDiffusiveTerm = ((1-this->timeImplicitCoefficient) * this->k * this->timeInterval) * this->diffusiveTerm[staggeredQuadrangle->getIndex()] * this->oldTemperature;
		this->linearSystem.independent[backElementIndex] += independentDiffusiveTerm;
		this->linearSystem.independent[frontElementIndex] += - independentDiffusiveTerm;
	}
	return;
}

ScalarStencil SquareCavityHeatTransfer::computeDiffusiveTerm(StaggeredElement2D& staggeredQuadrangle)
{
	Eigen::Vector3d areaVector = staggeredQuadrangle.getAreaVector();
	VectorStencil gradient = this->grid2D.computeVectorStencilOnQuadrangle(staggeredQuadrangle,this->scalarStencilOnVertices);
	return areaVector * gradient;
}

void SquareCavityHeatTransfer::applyBoundaryConditions(void)
{
	this->applyBoundaryConditionsToMatrix();
	this->applyBoundaryConditionsToIndependent();
	return;
}

void SquareCavityHeatTransfer::applyBoundaryConditionsToMatrix(void)
{
	for(auto& dirichlet: this->dirichletBoundaries)
		this->applyBoundaryConditionToMatrix(dirichlet);
	return;
}

void SquareCavityHeatTransfer::applyBoundaryConditionToMatrix(DirichletBoundaryCondition& dirichlet)
{
	for(unsigned i=0 ; i<dirichlet.staggeredTriangle.size() ; ++i)
		this->applyDirichletBoundaryConditionInStaggeredTriangleToMatrix(*(dirichlet.staggeredTriangle[i]), dirichlet.prescribedValue[i]);
}

void SquareCavityHeatTransfer::applyDirichletBoundaryConditionInStaggeredTriangleToMatrix(StaggeredElement2D& staggeredTriangle, const double prescribedValue)
{
	const unsigned elementIndex = staggeredTriangle.elements[0]->getIndex();
	Eigen::Vector3d gradientVector = staggeredTriangle.getCentroid() - staggeredTriangle.elements[0]->getCentroid();
	gradientVector = gradientVector / gradientVector.squaredNorm();
	double coeff = - staggeredTriangle.getAreaVector().dot(gradientVector) * this->k * this->timeInterval;
	this->linearSystem.matrix(elementIndex,elementIndex) += coeff * this->timeImplicitCoefficient;
	return;
}

void SquareCavityHeatTransfer::applyBoundaryConditionsToIndependent(void)
{
	for(auto& dirichlet: this->dirichletBoundaries)
		this->applyBoundaryConditionToIndependent(dirichlet);
	return;
}

void SquareCavityHeatTransfer::applyBoundaryConditionToIndependent(DirichletBoundaryCondition& dirichlet)
{
	for(unsigned i=0 ; i<dirichlet.staggeredTriangle.size() ; ++i)
		this->applyDirichletBoundaryConditionInStaggeredTriangleToIndependent(*(dirichlet.staggeredTriangle[i]), dirichlet.prescribedValue[i]);
}

void SquareCavityHeatTransfer::applyDirichletBoundaryConditionInStaggeredTriangleToIndependent(StaggeredElement2D& staggeredTriangle, const double prescribedValue)
{
	const unsigned elementIndex = staggeredTriangle.elements[0]->getIndex();
	Eigen::Vector3d gradientVector = staggeredTriangle.getCentroid() - staggeredTriangle.elements[0]->getCentroid();
	gradientVector = gradientVector / gradientVector.squaredNorm();
	double coeff = - staggeredTriangle.getAreaVector().dot(gradientVector) * this->k * this->timeInterval;
	this->linearSystem.independent(elementIndex) -= coeff * (1 - this->timeImplicitCoefficient) * this->oldTemperature[elementIndex];
	this->linearSystem.independent(elementIndex) += coeff * prescribedValue;
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