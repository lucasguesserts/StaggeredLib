#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>
#include <Eigen/Dense>
#include <stdexcept>

FacetCenterHeatTransfer::FacetCenterHeatTransfer(const std::string& fileName)
	: grid2D(fileName)
{
	this->initializeLinearSystem();
	this->initializeScalarStencilOnVertices();
	this->initializeScalarStencilOnElements();
	this->initializeGradientOnFaces();
	return;
}

void FacetCenterHeatTransfer::initializeLinearSystem(void)
{
	const unsigned numberOfStaggeredElements = this->grid2D.staggeredElements.size();
	this->linearSystem.matrix.resize(numberOfStaggeredElements,numberOfStaggeredElements);
	this->linearSystem.independent = Eigen::VectorXd::Zero(numberOfStaggeredElements);
	this->temperature = Eigen::VectorXd::Zero(numberOfStaggeredElements);
	return;
}

void FacetCenterHeatTransfer::initializeScalarStencilOnVertices(void)
{
	this->scalarStencilOnVertices = this->grid2D.computeScalarStencilOnVerticesUsingStaggeredElements();
	return;
}

void FacetCenterHeatTransfer::initializeScalarStencilOnElements(void)
{
	this->scalarStencilOnElements.resize(this->grid2D.elements.size());
	auto addScalarStencil = [this](StaggeredElement2D* staggeredElement, Element* element) -> void
	{
		const unsigned elementIndex = element->getIndex();
		const unsigned staggeredElementIndex = staggeredElement->getIndex();
		const double weight = 1.0 / static_cast<double>(element->vertices.size());
		this->scalarStencilOnElements[elementIndex] = this->scalarStencilOnElements[elementIndex] +
		                                              ScalarStencil{{staggeredElementIndex, weight}};
	};
	for(auto& staggeredElement: this->grid2D.staggeredQuadrangles)
	{
		addScalarStencil(staggeredElement, staggeredElement->elements[0]);
		addScalarStencil(staggeredElement, staggeredElement->elements[1]);
	}
	for(auto& staggeredElement: this->grid2D.staggeredTriangles)
		addScalarStencil(staggeredElement, staggeredElement->elements[0]);
	return;
}

void FacetCenterHeatTransfer::initializeGradientOnFaces(void)
{
	const unsigned numberOfFaces = this->grid2D.faces.size();
	this->gradientOnFaces.resize(numberOfFaces);
	for(Face2D& face: this->grid2D.faces)
	{
		Eigen::Vector3d frontBackDifferencePosition = face.forwardStaggeredElement->getCentroid() - face.backwardStaggeredElement->getCentroid();
		ScalarStencil frontBackDifferenceScalarStencil = ScalarStencil{
			                                             	{face.forwardStaggeredElement->getIndex(), +1.0},
			                                             	{face.backwardStaggeredElement->getIndex(), -1.0}};
		Eigen::Vector3d vertexElementDifferencePosition = *(face.adjacentVertex) - face.parentElement->getCentroid();
		ScalarStencil vertexElementDifferenceScalarStencil = this->scalarStencilOnVertices[face.adjacentVertex->getIndex()] +
		                                                     (-1) * this->scalarStencilOnElements[face.parentElement->getIndex()];
		this->gradientOnFaces[face.getIndex()] = ((1 / frontBackDifferencePosition.squaredNorm()) * frontBackDifferenceScalarStencil) * frontBackDifferencePosition +
		                                         ((1 / vertexElementDifferencePosition.squaredNorm()) * vertexElementDifferenceScalarStencil) * vertexElementDifferencePosition;
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
	this->applyBoundaryConditionsToMatrix();
	this->applyBoundaryConditionsToIndependent();
}

void FacetCenterHeatTransfer::applyBoundaryConditionsToMatrix(void)
{
	for(auto& dirichlet: this->dirichletBoundaries)
	{
		for(unsigned localIndex=0 ; localIndex<dirichlet.staggeredTriangle.size() ; ++localIndex)
		{
			auto& staggeredTriangle = dirichlet.staggeredTriangle[localIndex];
			const unsigned row = staggeredTriangle->getIndex();
			for(auto& triplet: this->linearSystem.coefficients)
			{
				if(triplet.row()==row)
					triplet = Eigen::Triplet<double,unsigned>(triplet.row(), triplet.col(), 0.0);
			}
			this->linearSystem.coefficients.push_back(Eigen::Triplet<double,unsigned>(row, row, 1.0));
		}
	}
	return;
}

void FacetCenterHeatTransfer::applyBoundaryConditionsToIndependent(void)
{
	for(auto& dirichlet: this->dirichletBoundaries)
	{
		for(unsigned localIndex=0 ; localIndex<dirichlet.staggeredTriangle.size() ; ++localIndex)
		{
			const unsigned row = dirichlet.staggeredTriangle[localIndex]->getIndex();
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