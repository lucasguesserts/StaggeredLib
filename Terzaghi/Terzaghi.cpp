#include <Terzaghi/Terzaghi.hpp>
#include <stdexcept>

const std::vector<Component> Terzaghi::displacementComponents = {Component::U, Component::V, Component::W};

Terzaghi::Terzaghi(const std::string& gridFile)
	: grid(gridFile)
{
	// Indices
	this->numberOfElements = this->grid.elements.size();
	this->numberOfStaggeredElements = this->grid.staggeredElements.size();
	this->transformToP = [](const unsigned index) -> unsigned
		{ return index; };
	this->transformToU = [&](const unsigned index) -> unsigned
		{ return this->numberOfElements + index; };
	this->transformToV = [&](const unsigned index) -> unsigned
		{ return this->numberOfElements + this->numberOfStaggeredElements + index; };
	this->transformToW = [&](const unsigned index) -> unsigned
		{ return this->numberOfElements + 2*(this->numberOfStaggeredElements) + index; };
	this->transform = { this->transformToP, this->transformToU, this->transformToV, this->transformToW };
	// Linear system
	this->linearSystemSize = this->numberOfElements + 3 * this->numberOfStaggeredElements;
	this->linearSystem.setSize(this->linearSystemSize);
	this->oldSolution.resize(this->linearSystemSize);
	// Pressure
	this->initializeScalarStencilOnVertices();
	this->initializePressureGradient();
	// Displacement
	this->initializeDisplacementScalarStencilOnElements();
	this->initializeDisplacementScalarStencilOnVertices();
	this->initializeDisplacementGradient();
	// Boundary
	this->initializeBoundaryConditions();
	return;
}

void Terzaghi::initializeScalarStencilOnVertices(void)
{
	this->scalarStencilOnVertices = this->grid.computeScalarStencilOnVertices();
}

void Terzaghi::initializePressureGradient(void)
{
	this->pressureGradient.resize(this->grid.staggeredElements.size());
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
		this->pressureGradient[staggeredQuadrangle->getIndex()] =
			this->grid.computeVectorStencilOnQuadrangle(*staggeredQuadrangle,this->scalarStencilOnVertices);
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		const unsigned elementIndex = staggeredTriangle->elements[0]->getIndex();
		Eigen::Vector3d gradientVector = staggeredTriangle->elements[0]->getCentroid() - staggeredTriangle->getCentroid();
		gradientVector = gradientVector / gradientVector.squaredNorm();
		this->pressureGradient[staggeredTriangle->getIndex()] = VectorStencil{{elementIndex, gradientVector}};
	}
	return;
}

void Terzaghi::initializeDisplacementScalarStencilOnElements(void)
{
	this->displacementScalarStencilOnElements.resize(this->grid.elements.size());
	auto addScalarStencil = [this](StaggeredElement2D* staggeredElement, Element* element) -> void
	{
		const unsigned elementIndex = element->getIndex();
		const unsigned staggeredElementIndex = staggeredElement->getIndex();
		const double weight = 1.0 / static_cast<double>(element->vertices.size());
		this->displacementScalarStencilOnElements[elementIndex] = this->displacementScalarStencilOnElements[elementIndex] +
		                                              ScalarStencil{{staggeredElementIndex, weight}};
	};
	for(auto& staggeredElement: this->grid.staggeredQuadrangles)
	{
		addScalarStencil(staggeredElement, staggeredElement->elements[0]);
		addScalarStencil(staggeredElement, staggeredElement->elements[1]);
	}
	for(auto& staggeredElement: this->grid.staggeredTriangles)
		addScalarStencil(staggeredElement, staggeredElement->elements[0]);
	return;
}

void Terzaghi::initializeDisplacementScalarStencilOnVertices(void)
{
	this->displacementScalarStencilOnVertices = this->grid.computeScalarStencilOnVerticesUsingStaggeredElements();
}

void Terzaghi::initializeDisplacementGradient(void)
{
	const unsigned numberOfFaces = this->grid.faces.size();
	this->displacementGradient.resize(numberOfFaces);
	for(Face2D& face: this->grid.faces)
	{
		Eigen::Vector3d frontBackDifferencePosition = face.forwardStaggeredElement->getCentroid() - face.backwardStaggeredElement->getCentroid();
		ScalarStencil frontBackDifferenceScalarStencil = ScalarStencil{
			                                             	{face.forwardStaggeredElement->getIndex(), +1.0},
			                                             	{face.backwardStaggeredElement->getIndex(), -1.0}};
		Eigen::Vector3d vertexElementDifferencePosition = *(face.adjacentVertex) - face.parentElement->getCentroid();
		ScalarStencil vertexElementDifferenceScalarStencil = this->displacementScalarStencilOnVertices[face.adjacentVertex->getIndex()] +
		                                                     (-1) * this->displacementScalarStencilOnElements[face.parentElement->getIndex()];
		this->displacementGradient[face.getIndex()] = ((1 / frontBackDifferencePosition.squaredNorm()) * frontBackDifferenceScalarStencil) * frontBackDifferencePosition +
		                                         ((1 / vertexElementDifferencePosition.squaredNorm()) * vertexElementDifferenceScalarStencil) * vertexElementDifferencePosition;
	}
	return;
}

void Terzaghi::initializeBoundaryConditions(void)
{
	std::string boundaryName;

	auto setBoundary = [this](
		const unsigned boundaryIndex,
		const std::string& boundaryName,
		BoundaryConditionType pressureBCtype,
		double pressureBCvalue,
		BoundaryConditionType uBCtype,
		double uBCvalue,
		BoundaryConditionType vBCtype,
		double vBCvalue,
		BoundaryConditionType wBCtype,
		double wBCvalue)
	{
		this->boundary[boundaryIndex].name = boundaryName;
		this->boundary[boundaryIndex].staggeredTriangle = this->grid.boundary[this->boundary[boundaryIndex].name].staggeredTriangle;
		if(this->boundary[boundaryIndex].staggeredTriangle.empty())
			throw std::runtime_error("Terzaghi boundary not set: " + boundaryName);

		this->boundary[boundaryIndex].component[0] = Component::P;
		this->boundary[boundaryIndex].boundaryConditionType[0] = pressureBCtype;
		this->boundary[boundaryIndex].prescribedValue[0] = pressureBCvalue;

		this->boundary[boundaryIndex].component[1] = Component::U;
		this->boundary[boundaryIndex].boundaryConditionType[1] = uBCtype;
		this->boundary[boundaryIndex].prescribedValue[1] = uBCvalue;

		this->boundary[boundaryIndex].component[2] = Component::V;
		this->boundary[boundaryIndex].boundaryConditionType[2] = vBCtype;
		this->boundary[boundaryIndex].prescribedValue[2] = vBCvalue;

		this->boundary[boundaryIndex].component[3] = Component::W;
		this->boundary[boundaryIndex].boundaryConditionType[3] = wBCtype;
		this->boundary[boundaryIndex].prescribedValue[3] = wBCvalue;
	};

	setBoundary( 0, "bottom boundary",
		BoundaryConditionType::Neumann, 0.0,
		BoundaryConditionType::Neumann, 0.0,
		BoundaryConditionType::Dirichlet, 0.0,
		BoundaryConditionType::Neumann, 0.0);

	setBoundary( 1, "top boundary",
		BoundaryConditionType::Dirichlet, 0.0,
		BoundaryConditionType::Neumann, 0.0,
		BoundaryConditionType::Neumann, 1.0E+6,
		BoundaryConditionType::Neumann, 0.0);

	setBoundary( 2, "east boundary",
		BoundaryConditionType::Neumann, 0.0,
		BoundaryConditionType::Dirichlet, 0.0,
		BoundaryConditionType::Neumann, 0.0,
		BoundaryConditionType::Neumann, 0.0);

	setBoundary( 3, "west boundary",
		BoundaryConditionType::Neumann, 0.0,
		BoundaryConditionType::Dirichlet, 0.0,
		BoundaryConditionType::Neumann, 0.0,
		BoundaryConditionType::Neumann, 0.0);

	return;
}

unsigned Terzaghi::transformIndex(const Component component, const unsigned index)
{
	return this->transform[static_cast<unsigned>(component)](index);
}

unsigned Terzaghi::transformIndex(const Component component, Entity* entity)
{
	return this->transformIndex(component, entity->getIndex());
}

Eigen::Vector3d Terzaghi::getDisplacementVector(StaggeredElement2D* staggeredElement)
{
	return Eigen::Vector3d(
		this->oldSolution[this->transformIndex(Component::U,staggeredElement)],
		this->oldSolution[this->transformIndex(Component::V,staggeredElement)],
		this->oldSolution[this->transformIndex(Component::W,staggeredElement)]
	);
}

void Terzaghi::insertPressureAccumulationTermInMatrix(void)
{
	const double compressibility = this->porosity * this->fluidCompressibility + (this->alpha - this->porosity) * this->solidCompressibility;
	for(auto& element: this->grid.elements)
	{
		const unsigned index = this->transformIndex(Component::P,element);
		this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(index, index, compressibility*element->getVolume()) );
	}
	return;
}

void Terzaghi::insertPressureDiffusiveTermInMatrix(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
	{
		const Eigen::Vector3d aux = (this->timeInterval * this->permeability / this->fluidViscosity *
		                            this->timeImplicitCoefficient) * staggeredQuadrangle->getAreaVector();
		ScalarStencil pressureDiffusionOnFace = aux * this->pressureGradient[staggeredQuadrangle->getIndex()];
		this->insertPressureScalarStencilInLinearSystem(staggeredQuadrangle->elements[0], pressureDiffusionOnFace); // front
		this->insertPressureScalarStencilInLinearSystem(staggeredQuadrangle->elements[1], (-1)*pressureDiffusionOnFace); // back
	}
	return;
}

void Terzaghi::insertPressureVolumeDilatationTermInMatrix(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
	{
		auto& coefficients = this->linearSystem.coefficients;
		coefficients.reserve(coefficients.size() + 6);
		Eigen::Vector3d areaVector = this->alpha * staggeredQuadrangle->getAreaVector();
		const unsigned frontRow = transformIndex(Component::P,staggeredQuadrangle->elements[0]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, transformIndex(Component::U,staggeredQuadrangle), -areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, transformIndex(Component::V,staggeredQuadrangle), -areaVector.y()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, transformIndex(Component::W,staggeredQuadrangle), -areaVector.z()) );
		const unsigned backRow = transformIndex(Component::P,staggeredQuadrangle->elements[1]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, transformIndex(Component::U,staggeredQuadrangle), areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, transformIndex(Component::V,staggeredQuadrangle), areaVector.y()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, transformIndex(Component::W,staggeredQuadrangle), areaVector.z()) );
	}
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		auto& coefficients = this->linearSystem.coefficients;
		coefficients.reserve(coefficients.size() + 3);
		Eigen::Vector3d areaVector = this->alpha * staggeredTriangle->getAreaVector();
		const unsigned frontRow = transformIndex(Component::P,staggeredTriangle->elements[0]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, transformIndex(Component::U,staggeredTriangle), -areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, transformIndex(Component::V,staggeredTriangle), -areaVector.y()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, transformIndex(Component::W,staggeredTriangle), -areaVector.z()) );
	}
	return;
}

void Terzaghi::insertDisplacementTensionTermInMatrix(void)
{
	for(auto& face: this->grid.faces)
	{
		for(auto forceComponent: this->displacementComponents)
		{
			for(auto displacementComponent: this->displacementComponents)
			{
				ScalarStencil solidForceOnFace = (face.getAreaVector().transpose() *
						this->getPermutationMatrix(forceComponent, displacementComponent) *
						this->getMechanicalPropertiesMatrix(forceComponent, displacementComponent)) *
						this->displacementGradient[face.getIndex()];
				insertScalarStencilDisplacementComponentInMatrix(forceComponent, displacementComponent, face.backwardStaggeredElement, solidForceOnFace);
				insertScalarStencilDisplacementComponentInMatrix(forceComponent, displacementComponent, face.forwardStaggeredElement, (-1)*solidForceOnFace);
			}
		}
	}
	return;
}

void Terzaghi::insertDisplacementPressureTermInMatrix(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
		for(auto forceComponent : this->displacementComponents)
			insertPressureGradientInMatrix(forceComponent, staggeredQuadrangle);
}

void Terzaghi::insertPressureGradientInMatrix(const Component forceComponent, StaggeredElement2D* staggeredQuadrangle)
{
	unsigned row = this->transformIndex(forceComponent, staggeredQuadrangle);
	const unsigned pressureVectorComponent = static_cast<unsigned>(forceComponent) - 1u;
	VectorStencil pressureTerm = (- this->alpha * staggeredQuadrangle->getVolume()) * this->pressureGradient[staggeredQuadrangle->getIndex()];
	for(auto keyValuePair: pressureTerm)
		this->linearSystem.coefficients.emplace_back(Eigen::Triplet<double,unsigned>(
			row,
			this->transformIndex(Component::P,keyValuePair.first),
			keyValuePair.second.coeff(pressureVectorComponent)));
	return;
}

void Terzaghi::insertPressureDiffusiveTermInIndependent(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
	{
		const Eigen::Vector3d aux = (this->timeInterval * this->permeability / this->fluidViscosity *
		                            (1 - this->timeImplicitCoefficient)) * staggeredQuadrangle->getAreaVector();
		ScalarStencil pressureDiffusionOnFace = aux * this->pressureGradient[staggeredQuadrangle->getIndex()];
		double independentValue = recoverPressureValueFromScalarStencil(pressureDiffusionOnFace);
		const unsigned frontRow = transformIndex(Component::P,staggeredQuadrangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
		const unsigned backRow = transformIndex(Component::P,staggeredQuadrangle->elements[1]);
		this->linearSystem.independent[backRow] += independentValue;
	}
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		const Eigen::Vector3d aux = (this->timeInterval * this->permeability / this->fluidViscosity *
		                            (1 - this->timeImplicitCoefficient)) * staggeredTriangle->getAreaVector();
		ScalarStencil pressureDiffusionOnFace = aux * this->pressureGradient[staggeredTriangle->getIndex()];
		double independentValue = recoverPressureValueFromScalarStencil(pressureDiffusionOnFace);
		const unsigned frontRow = transformIndex(Component::P,staggeredTriangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
	}
	return;
}

double Terzaghi::recoverPressureValueFromScalarStencil(const ScalarStencil& scalarStencilOnElements)
{
	double value = 0.0;
	for(auto& keyValuePair: scalarStencilOnElements)
	{
		const unsigned index = transformIndex(Component::P,keyValuePair.first);
		const double weightValue = keyValuePair.second;
		value += weightValue * this->oldSolution[index];
	}
	return value;
}

void Terzaghi::insertPressureScalarStencilInLinearSystem(Element* element, const ScalarStencil& scalarStencilOnElements)
{
	const unsigned row = transformIndex(Component::P,element);
	this->linearSystem.coefficients.reserve(this->linearSystem.coefficients.size() + scalarStencilOnElements.size());
	for(auto& keyValuePair: scalarStencilOnElements)
		this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, transformIndex(Component::P,keyValuePair.first), keyValuePair.second) );
	return;
}

void Terzaghi::insertScalarStencilDisplacementComponentInMatrix(const Component forceComponent, const Component displacementComponent, StaggeredElement2D* staggeredElement, const ScalarStencil& scalarStencilOnStaggeredElements)
{
	const unsigned row = this->transformIndex(forceComponent, staggeredElement);
	this->linearSystem.coefficients.reserve(this->linearSystem.coefficients.size() + scalarStencilOnStaggeredElements.size());
	for(auto& keyValuePair: scalarStencilOnStaggeredElements)
	{
		const unsigned column = this->transformIndex(displacementComponent, keyValuePair.first);
		this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, column, keyValuePair.second) );
	}
	return;
}

void Terzaghi::insertDisplacementDirichletBoundaryConditionToMatrix(void)
{
	for(auto boundary: this->boundary)
	{
		for(unsigned count=0 ; count<boundary.component.size() ; count++)
		{
			if(boundary.component[count]==Component::P) continue;
			if(boundary.boundaryConditionType[count]!=BoundaryConditionType::Dirichlet) continue;
			for(auto staggeredTriangle: boundary.staggeredTriangle)
				this->applyDisplacementDirichletBoundaryCondition(boundary.component[count], staggeredTriangle);
		}
	}
}

void Terzaghi::insertPressureAccumulationTermInIndependent(void)
{
	const double compressibility = this->porosity * this->fluidCompressibility + (this->alpha - this->porosity) * this->solidCompressibility;
	for(auto& element: this->grid.elements)
	{
		const unsigned index = this->transformIndex(Component::P,element);
		this->linearSystem.independent(index) = compressibility * element->getVolume() * this->oldSolution[index];
	}
	return;
}

void Terzaghi::insertPressureVolumeDilatationTermInIndependent(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
	{
		double independentValue = this->alpha * (staggeredQuadrangle->getAreaVector().dot(this->getDisplacementVector(staggeredQuadrangle)));
		const unsigned frontRow = transformIndex(Component::P,staggeredQuadrangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
		const unsigned backRow = transformIndex(Component::P,staggeredQuadrangle->elements[1]);
		this->linearSystem.independent[backRow] += independentValue;
	}
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		double independentValue = this->alpha * (staggeredTriangle->getAreaVector().dot(this->getDisplacementVector(staggeredTriangle)));
		const unsigned frontRow = transformIndex(Component::P,staggeredTriangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
	}
	return;
}

Eigen::MatrixXd Terzaghi::getPermutationMatrix(const Component c0, const Component c1)
{
	constexpr unsigned matrixSize = 3;
	const auto i = static_cast<unsigned>(c0) - 1u;
	const auto j = static_cast<unsigned>(c1) - 1u;
	Eigen::MatrixXd permutationMatrix = Eigen::MatrixXd::Zero(matrixSize,matrixSize);
	if(i==j)
		permutationMatrix = Eigen::MatrixXd::Identity(matrixSize,matrixSize);
	else
	{
		const unsigned k = 3 - (i + j);
		permutationMatrix(i,j) = 1;
		permutationMatrix(j,i) = 1;
		permutationMatrix(k,k) = 1;
	}
	return permutationMatrix;
}

Eigen::MatrixXd Terzaghi::getMechanicalPropertiesMatrix(const Component c0, const Component c1)
{
	constexpr unsigned matrixSize = 3;
	const auto i = static_cast<unsigned>(c0) - 1u;
	const auto j = static_cast<unsigned>(c1) - 1u;
	Eigen::MatrixXd mechanicalPropertiesMatrix = Eigen::MatrixXd::Zero(matrixSize,matrixSize);
	if(i==j)
	{
		for(unsigned k=0 ; k<matrixSize ; ++k)
			mechanicalPropertiesMatrix(k,k) = this->shearModulus;
		mechanicalPropertiesMatrix(i,i) *= 2 * (1 - this->poissonCoefficient) / (1 - 2*this->poissonCoefficient);
	}
	else
	{
		mechanicalPropertiesMatrix(i,i) = this->shearModulus;
		mechanicalPropertiesMatrix(j,j) = 2 * this->shearModulus * this->poissonCoefficient / (1 - 2*this->poissonCoefficient);
	}
	return mechanicalPropertiesMatrix;
}

VectorStencil Terzaghi::getDisplacementGradientOnStaggeredTriangle(StaggeredElement2D* staggeredTriangle)
{
	auto neighbor_0 = this->findStaggeredTriangleNeighbor(staggeredTriangle, staggeredTriangle->vertices[0], staggeredTriangle->elements[0]);
	auto neighbor_1 = this->findStaggeredTriangleNeighbor(staggeredTriangle, staggeredTriangle->vertices[1], staggeredTriangle->elements[0]);
	Eigen::Vector3d vector_0 = neighbor_0->getCentroid() - staggeredTriangle->getCentroid();
	vector_0 = vector_0 / vector_0.squaredNorm();
	Eigen::Vector3d vector_1 = neighbor_1->getCentroid() - staggeredTriangle->getCentroid();
	vector_1 = vector_1 / vector_1.squaredNorm();
	VectorStencil gradient = {
		{neighbor_0->getIndex(), vector_0},
		{neighbor_1->getIndex(), vector_1},
		{staggeredTriangle->getIndex(), -(vector_0 + vector_1)}
	};
	return gradient;
}

StaggeredElement2D* Terzaghi::findStaggeredTriangleNeighbor(StaggeredElement2D* staggeredTriangle, Vertex* adjacentVertex, Element* parentElement)
{
	std::array<unsigned,2> staggeredElementsLocation = this->grid.findStaggeredElements(adjacentVertex, parentElement);
	StaggeredElement2D* possibleNeighbor_0 = &(this->grid.staggeredElements[staggeredElementsLocation[0]]);
	StaggeredElement2D* possibleNeighbor_1 = &(this->grid.staggeredElements[staggeredElementsLocation[1]]);
	StaggeredElement2D* neighbor;
	if(possibleNeighbor_0!=staggeredTriangle)
		neighbor = possibleNeighbor_0;
	else
		neighbor = possibleNeighbor_1;
	return neighbor;
}

void Terzaghi::applyDisplacementDirichletBoundaryCondition(const Component component, StaggeredElement2D* staggeredTriangle)
{
	// TODO: move this function to somewhere in LinearSystem class
	const unsigned row = this->transformIndex(component, staggeredTriangle);
	for(auto& triplet: this->linearSystem.coefficients)
	{
		if(triplet.row()==row)
			triplet = Eigen::Triplet<double,unsigned>(triplet.row(), triplet.col(), 0.0);
	}
	this->linearSystem.coefficients.push_back(Eigen::Triplet<double,unsigned>(row, row, 1.0));
}

void Terzaghi::setOldPressure(const std::function<double(Eigen::Vector3d)> oldPressureFunction)
{
	for(auto& element: this->grid.elements)
	{
		const unsigned index = transformIndex(Component::P,element);
		this->oldSolution[index] = oldPressureFunction(element->getCentroid());
	}
	return;
}

void Terzaghi::setOldPressure(const std::vector<double> oldPressureValues)
{
	if(oldPressureValues.size()!=this->numberOfElements)
		throw std::runtime_error("pressure old values size differ from grid number of elements.");
	for(unsigned count = 0 ; count<(this->grid.elements.size()) ; ++count)
	{
		const unsigned index = transformIndex(Component::P, this->grid.elements[count] );
		this->oldSolution[index] = oldPressureValues[count];
	}
	return;
}

void Terzaghi::setOldDisplacement(const std::vector<Eigen::Vector3d>& displacements)
{
	if(displacements.size()!=this->numberOfStaggeredElements)
		throw std::runtime_error("Displacement old values size differ from grid number of staggered elements.");
	for(unsigned count = 0 ; count<(this->grid.staggeredElements.size()) ; ++count)
	{
		StaggeredElement2D* staggeredElement = &(this->grid.staggeredElements[count]);
		const unsigned uIndex = transformIndex(Component::U, staggeredElement );
		const unsigned vIndex = transformIndex(Component::V, staggeredElement );
		const unsigned wIndex = transformIndex(Component::W, staggeredElement );
		this->oldSolution[uIndex] = displacements[count].x();
		this->oldSolution[vIndex] = displacements[count].y();
		this->oldSolution[wIndex] = displacements[count].z();
	}
	return;
}