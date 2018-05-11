#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>
#include <stdexcept>

FacetCenterHeatTransfer::FacetCenterHeatTransfer(const GridData& gridData)
	: grid2D(gridData)
{
	this->initializeLinearSystem();
	this->initializeTemperatureVectors();
	this->initializeGradientOnFaces();
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

VectorStencil operator*(Eigen::MatrixXd gradientMatrix, std::vector<ScalarStencil> scalarStencilOnElementVertices)
{
	VectorStencil result;
	if(gradientMatrix.cols()!=static_cast<unsigned>(scalarStencilOnElementVertices.size()))
		throw std::runtime_error("Class FacetCenterHeatTransfer: gradientMatrix and scalarStencilOnElementVertices with incompatible sizes.");
	const unsigned numberOfColumns = gradientMatrix.cols();
	for(unsigned column=0 ; column<numberOfColumns ; ++column)
	{
		result = result + scalarStencilOnElementVertices[column] * Eigen::Vector3d(gradientMatrix.block(0,column,3,1));
	}
	return result;
}