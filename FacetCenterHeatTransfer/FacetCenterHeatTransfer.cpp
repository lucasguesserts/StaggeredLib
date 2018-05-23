#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>
#include <Eigen/Dense>
#include <stdexcept>

FacetCenterHeatTransfer::FacetCenterHeatTransfer(const std::string& fileName)
	: grid2D(fileName)
{
	this->initializeLinearSystem();
	this->initializeScalarStencilOnVertices();
	this->initializeGradientOnFaces();
	return;
}

void FacetCenterHeatTransfer::initializeLinearSystem(void)
{
	const unsigned numberOfStaggeredElements = this->grid2D.staggeredElements.size();
	this->linearSystem.matrix = Eigen::MatrixXd::Zero(numberOfStaggeredElements,numberOfStaggeredElements);
	this->linearSystem.independent = Eigen::VectorXd::Zero(numberOfStaggeredElements);
	this->temperature = Eigen::VectorXd::Zero(numberOfStaggeredElements);
	return;
}

void FacetCenterHeatTransfer::initializeScalarStencilOnVertices(void)
{
	this->scalarStencilOnVertices = this->grid2D.computeScalarStencilOnVerticesUsingStaggeredElements();
	return;
}

void FacetCenterHeatTransfer::initializeGradientOnFaces(void)
{
	const unsigned numberOfFaces = this->grid2D.faces.size();
	this->gradientOnFaces.resize(numberOfFaces);
	for(Face2D& face: this->grid2D.faces)
	{
		Eigen::MatrixXd gradientMatrix = this->computeGradientMatrix(face);
		std::vector<ScalarStencil> scalarStencilOnElementVertices = this->getScalarStencilOnElementVertices(face);
		this->gradientOnFaces[face.getIndex()] = gradientMatrix * scalarStencilOnElementVertices;
	}
	return;
}

void FacetCenterHeatTransfer::insertDirichletBoundaryCondition(const std::string& boundaryName, const std::function<double(Eigen::Vector3d)> prescribedValueFunction)
{
	DirichletBoundaryCondition dirichlet;
	dirichlet.staggeredTriangle = this->grid2D.boundary[boundaryName].staggeredTriangle;
	dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
	for(unsigned count=0 ; count<dirichlet.staggeredTriangle.size() ; ++count)
		dirichlet.prescribedValue[count] = prescribedValueFunction(dirichlet.staggeredTriangle[count]->getCentroid());
	this->dirichletBoundaries.emplace_back(std::move(dirichlet));
	return;
}

void FacetCenterHeatTransfer::insertDirichletBoundaryCondition(const std::string& boundaryName, const double prescribedValue)
{
	auto dirichletFunction = [prescribedValue](Eigen::Vector3d) -> double { return prescribedValue; };
	this->insertDirichletBoundaryCondition(boundaryName, dirichletFunction);
	return;
}

Eigen::MatrixXd FacetCenterHeatTransfer::computeGradientMatrix(Face2D& face)
{
	const Eigen::Vector3d faceLocalCoordinates = face.parentElement->getFaceLocalCoordinates(face.localIndex);
	return face.parentElement->getGradientMatrix2D(faceLocalCoordinates);
}

std::vector<ScalarStencil> FacetCenterHeatTransfer::getScalarStencilOnElementVertices(Face2D& face)
{
	std::vector<ScalarStencil> scalarStencilOnElementVertices;
	for(auto vertex: face.parentElement->vertices)
		scalarStencilOnElementVertices.push_back( this->scalarStencilOnVertices[vertex->getIndex()] );
	return scalarStencilOnElementVertices;
}

void FacetCenterHeatTransfer::addDiffusiveTerm(void)
{
	for(Face2D& face: this->grid2D.faces)
	{
		ScalarStencil heatDiffusion = face.getAreaVector() * this->gradientOnFaces[face.getIndex()];
		this->linearSystem.addScalarStencil(face.forwardStaggeredElement->getIndex(), (-1)*heatDiffusion);
		this->linearSystem.addScalarStencil(face.backwardStaggeredElement->getIndex(), heatDiffusion);
	}
	return;
}

void FacetCenterHeatTransfer::applyBoundaryConditions(void)
{
	for(auto& dirichlet: this->dirichletBoundaries)
	{
		for(unsigned localIndex=0 ; localIndex<dirichlet.staggeredTriangle.size() ; ++localIndex)
		{
			// TODO: separate into several functions.
			auto staggeredTriangle = dirichlet.staggeredTriangle[localIndex];
			const unsigned row = staggeredTriangle->getIndex();
			for(unsigned col=0 ; col<this->linearSystem.matrix.cols() ; ++col)
			{
				this->linearSystem.matrix(row,col) = 0.0; // TODO:consider create a "clear line" function at LinearSystem.
			}
			this->linearSystem.matrix(row,row) = 1.0;
			this->linearSystem.independent(row) = dirichlet.prescribedValue[localIndex];
		}
	}
	return;
}

VectorStencil operator*(Eigen::MatrixXd gradientMatrix, std::vector<ScalarStencil> scalarStencilOnElementVertices)
{
	VectorStencil result;
	if(gradientMatrix.cols()!=static_cast<unsigned>(scalarStencilOnElementVertices.size()))
		throw std::runtime_error("Class FacetCenterHeatTransfer: gradientMatrix and scalarStencilOnElementVertices with incompatible sizes.");
	const unsigned numberOfColumns = gradientMatrix.cols();
	for(unsigned column=0 ; column<numberOfColumns ; ++column)
	{
		result = result + scalarStencilOnElementVertices[column] * Eigen::Vector3d(gradientMatrix.block<3,1>(0,column));
	}
	return result;
}