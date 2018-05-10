#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>

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
	const unsigned numberOfVertices = face.parentElement->vertices.size();
	// TODO: complete it.
	return Eigen::MatrixXd::Zero(3,numberOfVertices);
}

std::vector<ScalarStencil> FacetCenterHeatTransfer::getScalarStencilOnElementVertices(Face2D& face)
{
	const unsigned numberOfVertices = face.parentElement->vertices.size();
	// TODO: complete it.
	return std::vector<ScalarStencil>{ {{0, numberOfVertices/3.14156535898}} };
}

VectorStencil operator*(Eigen::MatrixXd gradientMatrix, std::vector<ScalarStencil> scalarStencilOnElementVertices)
{
	// TODO: complete it.
	return VectorStencil{ {0,{0.0,0.0,0.0}} };
}