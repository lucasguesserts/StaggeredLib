#include <Terzaghi/Terzaghi.hpp>
#include <stdexcept>
#include <Eigen/Eigen>

const std::vector<Component> Terzaghi::displacementComponents = {Component::U, Component::V};

const std::vector<Eigen::Matrix<double,1,3>> Terzaghi::leftDisplacementMatrix = {
	{1, 0, 0},
	{0, 1, 0},
	{0, 0, 1}
};

const std::vector<Eigen::Matrix<double,6,3>> Terzaghi::rightDisplacementMatrix = {
	{
		(Eigen::Matrix<double,6,3>() <<
		1, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 1, 0,
		0, 0, 0,
		0, 0, 1
		).finished()
	},
	{
		(Eigen::Matrix<double,6,3>() <<
		0, 0, 0,
		0, 1, 0,
		0, 0, 0,
		1, 0, 0,
		0, 0, 1,
		0, 0, 0
		).finished()
	},
	{
		(Eigen::Matrix<double,6,3>() <<
		0, 0, 0,
		0, 0, 0,
		0, 0, 1,
		0, 0, 0,
		0, 1, 0,
		1, 0, 0
		).finished()
	}
};

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
	this->transform = { this->transformToP, this->transformToU, this->transformToV };
	// Linear system
	this->linearSystemSize = this->numberOfElements + 2 * this->numberOfStaggeredElements;
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
	this->pressureGradientIndependent.resize(this->grid.staggeredElements.size());
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
		this->pressureGradient[staggeredQuadrangle->getIndex()] =
			this->grid.computeVectorStencilOnQuadrangle(*staggeredQuadrangle,this->scalarStencilOnVertices);
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		const unsigned elementIndex = staggeredTriangle->elements[0]->getIndex();
		Eigen::Vector3d gradientVector = staggeredTriangle->elements[0]->getCentroid() - staggeredTriangle->getCentroid();
		gradientVector = gradientVector / gradientVector.squaredNorm();
		this->pressureGradient[staggeredTriangle->getIndex()] = VectorStencil{{elementIndex, gradientVector}};
		this->pressureGradientIndependent[staggeredTriangle->getIndex()] = - gradientVector;
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
		Eigen::Vector3d vertexElementDifferencePosition = *(face.adjacentVertex) - face.parentElement->getCentroid();
		const double frontBackNorm = frontBackDifferencePosition.norm();
		const double vertexElementNorm = vertexElementDifferencePosition.norm();
		ScalarStencil frontBackDifferenceScalarStencil = ScalarStencil{
			                                             	{face.forwardStaggeredElement->getIndex(), +1.0},
			                                             	{face.backwardStaggeredElement->getIndex(), -1.0}};
		ScalarStencil vertexElementDifferenceScalarStencil = this->displacementScalarStencilOnVertices[face.adjacentVertex->getIndex()] +
		                                                     (-1) * this->displacementScalarStencilOnElements[face.parentElement->getIndex()];
		// Compute vector stencil
		Eigen::Matrix<double,3,2> directionComponentsMatrix = Eigen::Matrix<double,3,2>::Zero();
			directionComponentsMatrix.block<1,2>(0,0) = frontBackDifferencePosition.normalized().block<2,1>(0,0);
			directionComponentsMatrix.block<1,2>(1,0) = vertexElementDifferencePosition.normalized().block<2,1>(0,0);
			directionComponentsMatrix.block<2,2>(0,0) = directionComponentsMatrix.block<2,2>(0,0).inverse().eval();
		Eigen::Matrix<double,2,2> inverseDistanceMatrix;
			inverseDistanceMatrix << 1.0/frontBackNorm, 0.0,
									0.0, 1.0/vertexElementNorm;
		Eigen::MatrixXd vectorsMatrix = directionComponentsMatrix * inverseDistanceMatrix;
		VectorStencil vectorStencil = frontBackDifferenceScalarStencil * Eigen::Vector3d(vectorsMatrix.block<3,1>(0,0)) +
		                              vertexElementDifferenceScalarStencil * Eigen::Vector3d(vectorsMatrix.block<3,1>(0,1));
		// set value
		this->displacementGradient[face.getIndex()] = vectorStencil;
	}
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
		0.0
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
		const unsigned frontRow = this->transformIndex(Component::P,staggeredQuadrangle->elements[0]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, this->transformIndex(Component::U,staggeredQuadrangle), -areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, this->transformIndex(Component::V,staggeredQuadrangle), -areaVector.y()) );
		const unsigned backRow = this->transformIndex(Component::P,staggeredQuadrangle->elements[1]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, this->transformIndex(Component::U,staggeredQuadrangle), areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, this->transformIndex(Component::V,staggeredQuadrangle), areaVector.y()) );
	}
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		auto& coefficients = this->linearSystem.coefficients;
		coefficients.reserve(coefficients.size() + 3);
		Eigen::Vector3d areaVector = - this->alpha * staggeredTriangle->getAreaVector();
		const unsigned row = this->transformIndex(Component::P,staggeredTriangle->elements[0]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, this->transformIndex(Component::U,staggeredTriangle), areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, this->transformIndex(Component::V,staggeredTriangle), areaVector.y()) );
	}
	return;
}

void Terzaghi::insertDisplacementTensionTermInMatrix(void)
{
	for(auto& face: this->grid.faces)
	{
		auto scalarStencilMatrix = this->computeDisplacementScalarStencilMatrix(face);
		for(auto forceComponent : this->displacementComponents)
		{
			for(auto displacementComponent : this->displacementComponents)
			{
				const unsigned i = static_cast<unsigned>(forceComponent) - 1;
				const unsigned j = static_cast<unsigned>(displacementComponent) - 1;
				this->insertScalarStencilDisplacementComponentInMatrix(forceComponent, displacementComponent, face.backwardStaggeredElement, scalarStencilMatrix[i][j]);
				this->insertScalarStencilDisplacementComponentInMatrix(forceComponent, displacementComponent, face.forwardStaggeredElement, (-1)*scalarStencilMatrix[i][j]);
			}
		}
	}
	return;
}

std::vector<std::vector<ScalarStencil>> Terzaghi::computeDisplacementScalarStencilMatrix(Face2D& face)
{
	std::vector<std::vector<ScalarStencil>> matrix(2, std::vector<ScalarStencil>(2));
	for(unsigned force=0 ; force<2 ; ++force)
	{
		for(unsigned displacement=0; displacement<2 ; ++displacement)
		{
			matrix[force][displacement] = ( this->leftDisplacementMatrix[force] *
			                              this->voigtTransformation(face.getAreaVector()).transpose() *
			                              this->getPhysicalPropertiesMatrix() *
			                              this->rightDisplacementMatrix[displacement] ) *
			                              this->displacementGradient[face.getIndex()];
		}
	}
	return matrix;
}

Eigen::MatrixXd Terzaghi::getPhysicalPropertiesMatrix(void)
// TODO: in some way, separate that code to use a one time defined value
{
	const double& G = this->shearModulus;
	const double& ni = this->poissonCoefficient;
	double lambda = 2 * G * ni / (1 - 2*ni);
	double diag = 2*G + lambda;
	return (Eigen::MatrixXd(6,6) <<
		diag  , lambda, 0, 0, 0, 0,
		lambda, diag  , 0, 0, 0, 0,
		0     , 0     , 0, 0, 0, 0,
		0     , 0     , 0, G, 0, 0,
		0     , 0     , 0, 0, 0, 0,
		0     , 0     , 0, 0, 0, 0
		).finished();
}

Eigen::MatrixXd Terzaghi::voigtTransformation(const Eigen::Vector3d& vector)
{
	return (Eigen::MatrixXd(6,3) <<
		vector.x(), 0         , 0         ,
		0         , vector.y(), 0         ,
		0         , 0         , vector.z(),
		vector.y(), vector.x(), 0         ,
		0         , vector.z(), vector.y(),
		vector.z(), 0         , vector.x()).finished();
}

void Terzaghi::insertDisplacementPressureTermInMatrix(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
		for(auto forceComponent : this->displacementComponents)
			insertPressureGradientInMatrix(forceComponent, staggeredQuadrangle);
}

void Terzaghi::insertPressureGradientInMatrix(const Component forceComponent, StaggeredElement2D* staggeredElement)
{
	unsigned row = this->transformIndex(forceComponent, staggeredElement);
	const unsigned pressureVectorComponent = static_cast<unsigned>(forceComponent) - 1u;
	VectorStencil pressureTerm = (- this->alpha * staggeredElement->getVolume()) * this->pressureGradient[staggeredElement->getIndex()];
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
		const unsigned frontRow = this->transformIndex(Component::P,staggeredQuadrangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
		const unsigned backRow = this->transformIndex(Component::P,staggeredQuadrangle->elements[1]);
		this->linearSystem.independent[backRow] += independentValue;
	}
	return;
}

double Terzaghi::recoverPressureValueFromScalarStencil(const ScalarStencil& scalarStencilOnElements)
{
	double value = 0.0;
	for(auto& keyValuePair: scalarStencilOnElements)
	{
		const unsigned index = this->transformIndex(Component::P,keyValuePair.first);
		const double weightValue = keyValuePair.second;
		value += weightValue * this->oldSolution[index];
	}
	return value;
}

void Terzaghi::insertPressureScalarStencilInLinearSystem(Element* element, const ScalarStencil& scalarStencilOnElements)
{
	const unsigned row = this->transformIndex(Component::P,element);
	this->linearSystem.coefficients.reserve(this->linearSystem.coefficients.size() + scalarStencilOnElements.size());
	for(auto& keyValuePair: scalarStencilOnElements)
		if(keyValuePair.second!=0.0)
			this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, this->transformIndex(Component::P,keyValuePair.first), keyValuePair.second) );
	return;
}

void Terzaghi::insertScalarStencilDisplacementComponentInMatrix(const Component forceComponent, const Component displacementComponent, StaggeredElement2D* staggeredElement, const ScalarStencil& scalarStencilOnStaggeredElements)
{
	const unsigned row = this->transformIndex(forceComponent, staggeredElement);
	this->linearSystem.coefficients.reserve(this->linearSystem.coefficients.size() + scalarStencilOnStaggeredElements.size());
	for(auto& keyValuePair: scalarStencilOnStaggeredElements)
	{
		const unsigned column = this->transformIndex(displacementComponent, keyValuePair.first);
		if(keyValuePair.second!=0.0)
			this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, column, keyValuePair.second) );
	}
	return;
}

void Terzaghi::insertForceComponentInIndependent(Eigen::Vector3d force, StaggeredElement2D* staggeredTriangle)
{
	for(auto forceComponent : this->displacementComponents)
	{
		auto component = static_cast<unsigned>(forceComponent) - 1u;
		const unsigned row = this->transformIndex(forceComponent, staggeredTriangle);
		this->linearSystem.independent[row] += - force[component];
	}
	return;
}

void Terzaghi::insertPressureAccumulationTermInIndependent(void)
{
	const double compressibility = this->porosity * this->fluidCompressibility + (this->alpha - this->porosity) * this->solidCompressibility;
	for(auto& element: this->grid.elements)
	{
		const unsigned index = this->transformIndex(Component::P,element);
		this->linearSystem.independent(index) += compressibility * element->getVolume() * this->oldSolution[index];
	}
	return;
}

void Terzaghi::insertPressureVolumeDilatationTermInIndependent(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
	{
		double independentValue = this->alpha * (staggeredQuadrangle->getAreaVector().dot(this->getDisplacementVector(staggeredQuadrangle)));
		const unsigned frontRow = this->transformIndex(Component::P,staggeredQuadrangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
		const unsigned backRow = this->transformIndex(Component::P,staggeredQuadrangle->elements[1]);
		this->linearSystem.independent[backRow] += independentValue;
	}
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		double independentValue = this->alpha * (staggeredTriangle->getAreaVector().dot(this->getDisplacementVector(staggeredTriangle)));
		const unsigned frontRow = this->transformIndex(Component::P,staggeredTriangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
	}
	return;
}

VectorStencil Terzaghi::getDisplacementGradientOnStaggeredTriangle(StaggeredElement2D* staggeredTriangle)
{
	auto neighbor_0 = this->findStaggeredTriangleNeighbor(staggeredTriangle, staggeredTriangle->vertices[0], staggeredTriangle->elements[0]);
	auto neighbor_1 = this->findStaggeredTriangleNeighbor(staggeredTriangle, staggeredTriangle->vertices[1], staggeredTriangle->elements[0]);
	Eigen::Vector3d vector_0 = neighbor_0->getCentroid() - staggeredTriangle->getCentroid();
	Eigen::Vector3d vector_1 = neighbor_1->getCentroid() - staggeredTriangle->getCentroid();
	ScalarStencil neighborScalarStencil_0 = ScalarStencil{ {neighbor_0->getIndex(), +1.0}, {staggeredTriangle->getIndex(), -1.0} };
	ScalarStencil neighborScalarStencil_1 = ScalarStencil{ {neighbor_1->getIndex(), +1.0}, {staggeredTriangle->getIndex(), -1.0} };
	// Compute vector stencil
	Eigen::Matrix<double,3,2> directionComponentsMatrix = Eigen::Matrix<double,3,2>::Zero();
		directionComponentsMatrix.block<1,2>(0,0) = vector_0.normalized().block<2,1>(0,0);
		directionComponentsMatrix.block<1,2>(1,0) = vector_1.normalized().block<2,1>(0,0);
		directionComponentsMatrix.block<2,2>(0,0) = directionComponentsMatrix.block<2,2>(0,0).inverse().eval();
	Eigen::Matrix<double,2,2> inverseDistanceMatrix;
		inverseDistanceMatrix << 1.0/vector_0.norm(), 0.0,
		                         0.0,                 1.0/vector_1.norm();
	Eigen::MatrixXd vectorsMatrix = directionComponentsMatrix * inverseDistanceMatrix;
	VectorStencil gradient = neighborScalarStencil_0 * Eigen::Vector3d(vectorsMatrix.block<3,1>(0,0)) +
	                              neighborScalarStencil_1 * Eigen::Vector3d(vectorsMatrix.block<3,1>(0,1));
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

void Terzaghi::setOldPressure(const std::function<double(Eigen::Vector3d)> oldPressureFunction)
{
	for(auto& element: this->grid.elements)
	{
		const unsigned index = this->transformIndex(Component::P,element);
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
		const unsigned index = this->transformIndex(Component::P, this->grid.elements[count] );
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
		const unsigned uIndex = this->transformIndex(Component::U, staggeredElement );
		const unsigned vIndex = this->transformIndex(Component::V, staggeredElement );
		this->oldSolution[uIndex] = displacements[count].x();
		this->oldSolution[vIndex] = displacements[count].y();
	}
	return;
}

void Terzaghi::initializeBoundaryConditions(void)
{
	this->boundaries.resize(4);
	std::string boundaryName;
	{
		boundaryName = "top boundary";
		TerzaghiBoundary& boundary = this->boundaries[0];
		boundary = this->boundaries[0];
		boundary.staggeredTriangles = this->grid.boundary[boundaryName].staggeredTriangle;
		boundary.stress << 0.0, -1E+6, 0.0, +0.0, 0.0, 0.0;
		boundary.isStressPrescribed = {false, true, false, true, false, false};
		boundary.applyTerzaghiPressureInStaggeredTriangles = true;
		boundary.prescribedDisplacement = {{ {false, 0.0}, {false, 0.0}, {false, 0.0} }};
		boundary.isPressureDirichlet = true;
		boundary.pressureGradient << 0.0, 0.0, 0.0;
		boundary.pressurePrescribedValue = 0.0;
	}
	{
		boundaryName = "bottom boundary";
		TerzaghiBoundary& boundary = this->boundaries[1];
		boundary.staggeredTriangles = this->grid.boundary[boundaryName].staggeredTriangle;
		boundary.stress << 0.0, 0.0, 0.0, +0.0, 0.0, 0.0;
		boundary.isStressPrescribed = {false, false, false, true, false, false};
		boundary.applyTerzaghiPressureInStaggeredTriangles = false;
		boundary.prescribedDisplacement = {{ {false, 0.0}, {true, 0.0}, {false, 0.0} }};
		boundary.isPressureDirichlet = false;
		boundary.pressureGradient << 0.0, 0.0, 0.0;
		boundary.pressurePrescribedValue = 0.0;
	}
	{
		boundaryName = "west boundary";
		TerzaghiBoundary& boundary = this->boundaries[2];
		boundary.staggeredTriangles = this->grid.boundary[boundaryName].staggeredTriangle;
		boundary.stress << 0.0, 0.0, 0.0, +0.0, 0.0, 0.0;
		boundary.isStressPrescribed = {false, false, false, true, false, false};
		boundary.applyTerzaghiPressureInStaggeredTriangles = false;
		boundary.prescribedDisplacement = {{ {true, 0.0}, {false, 0.0}, {false, 0.0} }};
		boundary.isPressureDirichlet = false;
		boundary.pressureGradient << 0.0, 0.0, 0.0;
		boundary.pressurePrescribedValue = 0.0;
	}
	{
		boundaryName = "east boundary";
		TerzaghiBoundary& boundary = this->boundaries[3];
		boundary.staggeredTriangles = this->grid.boundary[boundaryName].staggeredTriangle;
		boundary.stress << 0.0, 0.0, 0.0, +0.0, 0.0, 0.0;
		boundary.isStressPrescribed = {false, false, false, true, false, false};
		boundary.applyTerzaghiPressureInStaggeredTriangles = false;
		boundary.prescribedDisplacement = {{ {true, 0.0}, {false, 0.0}, {false, 0.0} }};
		boundary.isPressureDirichlet = false;
		boundary.pressureGradient << 0.0, 0.0, 0.0;
		boundary.pressurePrescribedValue = 0.0;
	}
	return;
}

void Terzaghi::insertPrescribedStressInIndependent(void)
{
	for(auto& boundary: this->boundaries)
	{
		for(auto staggeredTriangle: boundary.staggeredTriangles)
		{
			Eigen::MatrixXd transformedAreaVector = this->voigtTransformation(- staggeredTriangle->getAreaVector()).transpose();
			Eigen::Vector3d force = transformedAreaVector * boundary.stress;
			for(auto forceComponent : this->displacementComponents)
			{
				const auto component = static_cast<unsigned>(forceComponent) - 1;
				auto row = this->transformIndex(forceComponent,staggeredTriangle);
				this->linearSystem.independent[row] += - force(component);
			}
		}
	}
}

// void Terzaghi::insertDisplacementBoundaryTensionTermInMatrix(void)
// {
// 	for(auto& boundary: this->boundaries)
// 	{
// 		for(auto staggeredTriangle: boundary.staggeredTriangles)
// 		{
// 			auto physicalPropertiesMatrix = this->getNeumannAppliedPhysicalPropertiesMatrix(boundary.isStressPrescribed);
// 			auto scalarStencilMatrix = this->computeDisplacementScalarStencilMatrix(staggeredTriangle, physicalPropertiesMatrix);
// 			for(auto forceComponent : this->displacementComponents)
// 			{
// 				for(auto displacementComponent : this->displacementComponents)
// 				{
// 					const unsigned i = static_cast<unsigned>(forceComponent) - 1;
// 					const unsigned j = static_cast<unsigned>(displacementComponent) - 1;
// 					this->insertScalarStencilDisplacementComponentInMatrix(forceComponent, displacementComponent, staggeredTriangle, scalarStencilMatrix[i][j]);
// 				}
// 			}
// 		}
// 	}
// }

// Eigen::MatrixXd Terzaghi::getNeumannAppliedPhysicalPropertiesMatrix(std::array<bool,6>& isStressPrescribed)
// {
// 	auto physicalPropertiesMatrix = this->getPhysicalPropertiesMatrix();
// 	for(unsigned i=0 ; i<6 ; ++i)
// 		if(isStressPrescribed[i])
// 			physicalPropertiesMatrix.block<1,6>(i,0) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
// 	return physicalPropertiesMatrix;
// }

// std::vector<std::vector<ScalarStencil>> Terzaghi::computeDisplacementScalarStencilMatrix(StaggeredElement2D* staggeredTriangle, Eigen::MatrixXd& physicalPropertiesMatrix)
// {
// 	std::vector<std::vector<ScalarStencil>> matrix(3, std::vector<ScalarStencil>(3));
// 	VectorStencil gradientDisplacement = this->getDisplacementGradientOnStaggeredTriangle(staggeredTriangle);
// 	for(unsigned force=0 ; force<3 ; ++force)
// 	{
// 		for(unsigned displacement=0; displacement<3 ; ++displacement)
// 		{
// 			matrix[force][displacement] = ( this->leftDisplacementMatrix[force] *
// 			                              this->voigtTransformation(-staggeredTriangle->getAreaVector()).transpose() *
// 			                              physicalPropertiesMatrix *
// 			                              this->rightDisplacementMatrix[displacement] ) *
// 			                              gradientDisplacement;
// 		}
// 	}
// 	return matrix;
// }

void Terzaghi::insertDisplacementDirichletBoundaryConditionToMatrix(void)
{
	for(auto& boundary: this->boundaries)
	{
		for(auto displacementComponent : this->displacementComponents)
		{
			const unsigned i = static_cast<unsigned>(displacementComponent) - 1;
			if(boundary.prescribedDisplacement[i].first)
				for(auto staggeredTriangle: boundary.staggeredTriangles)
					this->applyDisplacementDirichletBoundaryCondition(displacementComponent, staggeredTriangle);
		}
	}
}

void Terzaghi::applyDisplacementDirichletBoundaryCondition(const Component component, StaggeredElement2D* staggeredTriangle)
{
	// TODO: update and move this function to somewhere in LinearSystem class
	const unsigned row = this->transformIndex(component, staggeredTriangle);
	bool wasInserted = false;
	for(auto& triplet: this->linearSystem.coefficients)
	{
		if(triplet.row()==row)
		{
			if(triplet.col()!=row || wasInserted)
				triplet = Eigen::Triplet<double,unsigned>(row, triplet.col(), 0.0);
			else
			{
				triplet = Eigen::Triplet<double,unsigned>(row, row, 1.0);
				wasInserted = true;
			}
		}
	}
	return;
}

void Terzaghi::insertDisplacementDirichletBoundaryConditionToIndependent(void)
{
	for(auto& boundary: this->boundaries)
	{
		for(auto displacementComponent : this->displacementComponents)
		{
			const unsigned i = static_cast<unsigned>(displacementComponent) - 1;
			if(boundary.prescribedDisplacement[i].first)
			{
				for(auto staggeredTriangle: boundary.staggeredTriangles)
				{
					const unsigned row = this->transformIndex(displacementComponent, staggeredTriangle);
					this->linearSystem.independent[row] = boundary.prescribedDisplacement[i].second;
				}
			}
		}
	}
	return;
}

void Terzaghi::insertDisplacementPressureDirichletBoundaryConditionToMatrix(void)
{
	for(auto& boundary: this->boundaries)
		if(boundary.isPressureDirichlet)
			for(auto staggeredTriangle: boundary.staggeredTriangles)
				for(auto forceComponent : this->displacementComponents)
					this->insertPressureGradientInMatrix(forceComponent, staggeredTriangle);
	return;
}

void Terzaghi::insertDisplacementPressureDirichletBoundaryConditionToIndependent(void)
{
	for(auto& boundary: this->boundaries)
		if(boundary.isPressureDirichlet)
			for(auto staggeredTriangle: boundary.staggeredTriangles)
				for(auto forceComponent : this->displacementComponents)
					this->insertPressureGradientInIndependent(forceComponent, staggeredTriangle, boundary.pressurePrescribedValue);
}

void Terzaghi::insertPressureGradientInIndependent(const Component forceComponent, StaggeredElement2D* staggeredElement, double prescribedValue)
{
	unsigned row = this->transformIndex(forceComponent, staggeredElement);
	const unsigned pressureVectorComponent = static_cast<unsigned>(forceComponent) - 1u;
	Eigen::Vector3d pressureTerm = ( this->alpha * staggeredElement->getVolume() *
	                               prescribedValue) *
	                               this->pressureGradientIndependent[staggeredElement->getIndex()];
	this->linearSystem.independent[row] += pressureTerm[pressureVectorComponent];
	return;
}

void Terzaghi::insertDisplacementPressureNeumannBoundaryConditionToIndependent(void)
{
	for(auto& boundary: this->boundaries)
		if( ! boundary.isPressureDirichlet )
			for(auto staggeredTriangle: boundary.staggeredTriangles)
				for(auto forceComponent : this->displacementComponents)
					this->insertPressureGradientNeumannInIndependent(forceComponent, staggeredTriangle, boundary.pressureGradient);
}

void Terzaghi::insertPressureGradientNeumannInIndependent(const Component forceComponent, StaggeredElement2D* staggeredElement, Eigen::Vector3d pressureGradientPrescribed)
{
	unsigned row = this->transformIndex(forceComponent, staggeredElement);
	const unsigned pressureVectorComponent = static_cast<unsigned>(forceComponent) - 1u;
	Eigen::Vector3d pressureTerm = ( this->alpha * staggeredElement->getVolume()) * pressureGradientPrescribed;
	this->linearSystem.independent[row] += pressureTerm[pressureVectorComponent];
	return;
}

void Terzaghi::insertPressureDirichletBoundaryConditionToMatrix(void)
{
	for(auto& boundary: this->boundaries)
		if(boundary.isPressureDirichlet)
			for(auto staggeredTriangle: boundary.staggeredTriangles)
			{
				const Eigen::Vector3d aux = (this->timeInterval * this->permeability / this->fluidViscosity *
											this->timeImplicitCoefficient) * ( - staggeredTriangle->getAreaVector());
				ScalarStencil pressureDiffusionOnFace = aux * this->pressureGradient[staggeredTriangle->getIndex()];
				this->insertPressureScalarStencilInLinearSystem(staggeredTriangle->elements[0], (-1)*pressureDiffusionOnFace); // back
			}
	return;
}

void Terzaghi::insertPressureDirichletBoundaryConditionToIndependent(void)
{
	for(auto& boundary: this->boundaries)
		if(boundary.isPressureDirichlet)
		{
			// Old diffusion
			for(auto staggeredTriangle: boundary.staggeredTriangles)
			{
				const Eigen::Vector3d aux = - (this->timeInterval * this->permeability / this->fluidViscosity *
											(1 - this->timeImplicitCoefficient)) * staggeredTriangle->getAreaVector();
				ScalarStencil pressureDiffusionOnFace = aux * this->pressureGradient[staggeredTriangle->getIndex()];
				const double independentValue = this->recoverPressureValueFromScalarStencil(pressureDiffusionOnFace);
				const unsigned row = this->transformIndex(Component::P,staggeredTriangle->elements[0]);
				this->linearSystem.independent[row] += independentValue;
			}
			// Prescribed value
			for(auto staggeredTriangle: boundary.staggeredTriangles)
			{
				const Eigen::Vector3d aux = (this->timeInterval * this->permeability / this->fluidViscosity * ( - staggeredTriangle->getAreaVector()));
				const double independentValue = boundary.pressurePrescribedValue *
				                                aux.dot( pressureGradientIndependent[staggeredTriangle->getIndex()] );
				const unsigned row = this->transformIndex(Component::P,staggeredTriangle->elements[0]);
				this->linearSystem.independent[row] += independentValue;
			}
		}
	return;
}

void Terzaghi::insertPressureNeumannBoundaryConditionToIndependent(void)
{
	for(auto& boundary: this->boundaries)
		if( ! boundary.isPressureDirichlet )
			for(auto staggeredTriangle: boundary.staggeredTriangles)
			{
				const double independentValue = (this->timeInterval * this->permeability / this->fluidViscosity) *
				                                ( - staggeredTriangle->getAreaVector()).dot( boundary.pressureGradient );
				const unsigned row = this->transformIndex(Component::P,staggeredTriangle->elements[0]);
				this->linearSystem.independent[row] += independentValue;
			}
	return;
}

void Terzaghi::insertPrescribedStressPressureTermInMatrix(void)
{
	for(auto& boundary: this->boundaries)
	{
		if(boundary.applyTerzaghiPressureInStaggeredTriangles)
		{
			for(auto staggeredTriangle: boundary.staggeredTriangles)
			{
				// vertex 0
				ScalarStencil pressure_0 = 0.5 * (this->scalarStencilOnVertices[staggeredTriangle->vertices[0]->getIndex()] +
				                                  ScalarStencil{ {staggeredTriangle->elements[0]->getIndex(), 1.0} });
				Eigen::Vector3d areaVector_0 = (staggeredTriangle->elements[0]->getCentroid()) - *(staggeredTriangle->vertices[0]);
					std::swap(areaVector_0[0],areaVector_0[1]);
					areaVector_0[1] = - areaVector_0[1];
				VectorStencil force_0 = pressure_0 * areaVector_0;
				this->insertPressureGradientInMatrix(staggeredTriangle, (-1)*force_0);
				// vertex 1
				ScalarStencil pressure_1 = 0.5 * (this->scalarStencilOnVertices[staggeredTriangle->vertices[1]->getIndex()] +
				                                  ScalarStencil{ {staggeredTriangle->elements[0]->getIndex(), 1.0} });
				Eigen::Vector3d areaVector_1 = *(staggeredTriangle->vertices[1]) - (staggeredTriangle->elements[0]->getCentroid());
					std::swap(areaVector_1[0],areaVector_1[1]);
					areaVector_1[1] = - areaVector_1[1];
				VectorStencil force_1 = pressure_1 * areaVector_1;
				this->insertPressureGradientInMatrix(staggeredTriangle, (-1)*force_1);
			}
		}
	}
	return;
}

void Terzaghi::insertPressureGradientInMatrix(StaggeredElement2D* staggeredElement, const VectorStencil& pressureTerm)
{
	for(auto forceComponent : this->displacementComponents)
	{
		unsigned row = this->transformIndex(forceComponent, staggeredElement);
		const unsigned pressureVectorComponent = static_cast<unsigned>(forceComponent) - 1u;
		for(auto keyValuePair: pressureTerm)
			this->linearSystem.coefficients.emplace_back(Eigen::Triplet<double,unsigned>(
				row,
				this->transformIndex(Component::P,keyValuePair.first),
				keyValuePair.second.coeff(pressureVectorComponent)));
	}
	return;
}

void Terzaghi::assemblyLinearSystemMatrix(void)
{
	this->insertPressureAccumulationTermInMatrix();
	this->insertPressureDiffusiveTermInMatrix();
	this->insertPressureVolumeDilatationTermInMatrix();

	this->insertDisplacementTensionTermInMatrix();
	this->insertDisplacementPressureTermInMatrix();

	this->insertPressureDirichletBoundaryConditionToMatrix();

	// this->insertDisplacementBoundaryTensionTermInMatrix();
	// this->insertDisplacementPressureDirichletBoundaryConditionToMatrix();
	this->insertPrescribedStressPressureTermInMatrix();
	this->insertDisplacementDirichletBoundaryConditionToMatrix();

	this->linearSystem.computeLU();

	return;
}

void Terzaghi::assemblyLinearSystemIndependent(void)
{
	this->linearSystem.independent = Eigen::VectorXd::Zero(this->linearSystemSize);

	this->insertPressureAccumulationTermInIndependent();
	this->insertPressureDiffusiveTermInIndependent();
	this->insertPressureVolumeDilatationTermInIndependent();

	this->insertPressureDirichletBoundaryConditionToIndependent();
	this->insertPressureNeumannBoundaryConditionToIndependent();

	this->insertPrescribedStressInIndependent();
	this->insertDisplacementPressureNeumannBoundaryConditionToIndependent();
	this->insertDisplacementPressureDirichletBoundaryConditionToIndependent();
	this->insertDisplacementDirichletBoundaryConditionToIndependent();

	return;
}

void Terzaghi::solve(void)
{
	this->assemblyLinearSystemIndependent();
	this->oldSolution = this->linearSystem.solve();
	return;
}

std::vector<double> Terzaghi::getComponentFromOldSolution(const Component component)
{
	std::vector<double> oldSolutionComponent;
	if(component==Component::P)
	{
		oldSolutionComponent.resize(this->numberOfElements);
		for(auto element: this->grid.elements)
		{
			const unsigned index = this->transformIndex(Component::P, element);
			oldSolutionComponent[element->getIndex()] = this->oldSolution[index];
		}
	}
	else
	{
		oldSolutionComponent.resize(this->numberOfStaggeredElements);
		for(auto staggeredElement: this->grid.staggeredElements)
		{
			const unsigned index = this->transformIndex(component, staggeredElement.getIndex());
			oldSolutionComponent[staggeredElement.getIndex()] = this->oldSolution[index];
		}
	}
	return oldSolutionComponent;
}