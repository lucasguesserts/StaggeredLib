#include <Terzaghi/Terzaghi.hpp>
#include <stdexcept>

Terzaghi::Terzaghi(const std::string& gridFile)
	: grid(gridFile)
{
	// Linear system
	this->numberOfElements = this->grid.elements.size();
	this->numberOfStaggeredElements = this->grid.staggeredElements.size();
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

unsigned Terzaghi::transformIndex(const unsigned component, const unsigned index)
{
	static auto transformToP = [](const unsigned index) -> unsigned
		{ return index; };
	static auto transformToU = [this](const unsigned index) -> unsigned
		{ return this->numberOfElements + index; };
	static auto transformToV = [this](const unsigned index) -> unsigned
		{ return this->numberOfElements + this->numberOfStaggeredElements + index; };
	static auto transformToW = [this](const unsigned index) -> unsigned
		{ return this->numberOfElements + 2*(this->numberOfStaggeredElements) + index; };
	static std::vector<std::function<unsigned(const unsigned)>> transform =
		{ transformToP, transformToU, transformToV, transformToW } ;
	return transform[component](index);
}

unsigned Terzaghi::getPindex(Element* element)
{
	return element->getIndex();
}

unsigned Terzaghi::getPindex(const unsigned elementIndex)
{
	return elementIndex;
}

unsigned Terzaghi::getUindex(StaggeredElement2D* staggeredElement)
{
	return this->numberOfElements + staggeredElement->getIndex();
}

unsigned Terzaghi::getUindex(const unsigned staggeredElementIndex)
{
	return this->numberOfElements + staggeredElementIndex;
}

unsigned Terzaghi::getVindex(StaggeredElement2D* staggeredElement)
{
	return this->numberOfElements + this->numberOfStaggeredElements + staggeredElement->getIndex();
}

unsigned Terzaghi::getVindex(const unsigned staggeredElementIndex)
{
	return this->numberOfElements + this->numberOfStaggeredElements + staggeredElementIndex;
}

unsigned Terzaghi::getWindex(StaggeredElement2D* staggeredElement)
{
	return this->numberOfElements + (2 * this->numberOfStaggeredElements) + staggeredElement->getIndex();
}

unsigned Terzaghi::getWindex(const unsigned staggeredElementIndex)
{
	return this->numberOfElements + (2 * this->numberOfStaggeredElements) + staggeredElementIndex;
}

Eigen::Vector3d Terzaghi::getDisplacementVector(StaggeredElement2D* staggeredElement)
{
	return Eigen::Vector3d(
		this->oldSolution[this->getUindex(staggeredElement)],
		this->oldSolution[this->getVindex(staggeredElement)],
		this->oldSolution[this->getWindex(staggeredElement)]
	);
}

void Terzaghi::insertPressureAccumulationTermInMatrix(void)
{
	const double compressibility = this->porosity * this->fluidCompressibility + (this->alpha - this->porosity) * this->solidCompressibility;
	for(auto& element: this->grid.elements)
	{
		const unsigned index = this->getPindex(element);
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
		const unsigned frontRow = getPindex(staggeredQuadrangle->elements[0]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, getUindex(staggeredQuadrangle), -areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, getVindex(staggeredQuadrangle), -areaVector.y()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, getWindex(staggeredQuadrangle), -areaVector.z()) );
		const unsigned backRow = getPindex(staggeredQuadrangle->elements[1]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, getUindex(staggeredQuadrangle), areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, getVindex(staggeredQuadrangle), areaVector.y()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(backRow, getWindex(staggeredQuadrangle), areaVector.z()) );
	}
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		auto& coefficients = this->linearSystem.coefficients;
		coefficients.reserve(coefficients.size() + 3);
		Eigen::Vector3d areaVector = this->alpha * staggeredTriangle->getAreaVector();
		const unsigned frontRow = getPindex(staggeredTriangle->elements[0]);
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, getUindex(staggeredTriangle), -areaVector.x()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, getVindex(staggeredTriangle), -areaVector.y()) );
			coefficients.emplace_back( Eigen::Triplet<double,unsigned>(frontRow, getWindex(staggeredTriangle), -areaVector.z()) );
	}
	return;
}

void Terzaghi::insertDisplacementTensionTermInMatrix(void)
{
	for(auto& face: this->grid.faces)
	{
		for(unsigned forceComponent=0 ; forceComponent<DisplacementIndex::numberOfComponents ; ++forceComponent)
		{
			for(unsigned displacementComponent=0 ; displacementComponent<DisplacementIndex::numberOfComponents ; ++displacementComponent)
			{
				ScalarStencil solidForceOnFace = (face.getAreaVector().transpose() *
						this->getPermutationMatrix(forceComponent,displacementComponent) *
						this->getMechanicalPropertiesMatrix(forceComponent,displacementComponent)) *
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
	{
		for(unsigned forceComponent=0 ; forceComponent<DisplacementIndex::numberOfComponents ; ++forceComponent)
		{
			insertPressureGradientInMatrix(forceComponent, staggeredQuadrangle);
		}
	}
}

void Terzaghi::insertPressureGradientInMatrix(const unsigned forceComponent, StaggeredElement2D* staggeredQuadrangle)
{
	unsigned row;
	VectorStencil pressureTerm = (- this->alpha * staggeredQuadrangle->getVolume()) * this->pressureGradient[staggeredQuadrangle->getIndex()];
	switch(forceComponent)
	{
		case DisplacementIndex::U:
			row = getUindex(staggeredQuadrangle);
			for(auto keyValuePair: pressureTerm)
				this->linearSystem.coefficients.emplace_back(Eigen::Triplet<double,unsigned>(row, this->getPindex(keyValuePair.first), keyValuePair.second.x()));
			break;
		case DisplacementIndex::V:
			row = getVindex(staggeredQuadrangle);
			for(auto keyValuePair: pressureTerm)
				this->linearSystem.coefficients.emplace_back(Eigen::Triplet<double,unsigned>(row, this->getPindex(keyValuePair.first), keyValuePair.second.y()));
			break;
		case DisplacementIndex::W:
			row = getWindex(staggeredQuadrangle);
			for(auto keyValuePair: pressureTerm)
				this->linearSystem.coefficients.emplace_back(Eigen::Triplet<double,unsigned>(row, this->getPindex(keyValuePair.first), keyValuePair.second.z()));
			break;
	}
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
		const unsigned frontRow = getPindex(staggeredQuadrangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
		const unsigned backRow = getPindex(staggeredQuadrangle->elements[1]);
		this->linearSystem.independent[backRow] += independentValue;
	}
	return;
}

double Terzaghi::recoverPressureValueFromScalarStencil(const ScalarStencil& scalarStencilOnElements)
{
	double value = 0.0;
	for(auto& keyValuePair: scalarStencilOnElements)
	{
		const unsigned index = getPindex(keyValuePair.first);
		const double weightValue = keyValuePair.second;
		value += weightValue * this->oldSolution[index];
	}
	return value;
}

void Terzaghi::insertPressureScalarStencilInLinearSystem(Element* element, const ScalarStencil& scalarStencilOnElements)
{
	const unsigned row = getPindex(element);
	this->linearSystem.coefficients.reserve(this->linearSystem.coefficients.size() + scalarStencilOnElements.size());
	for(auto& keyValuePair: scalarStencilOnElements)
		this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, getPindex(keyValuePair.first), keyValuePair.second) );
	return;
}

void Terzaghi::insertScalarStencilDisplacementComponentInMatrix(const unsigned forceComponent, const unsigned displacementComponent, StaggeredElement2D* staggeredElement, const ScalarStencil& scalarStencilOnStaggeredElements)
{
	unsigned row;
	switch(forceComponent)
	{
		case DisplacementIndex::U:
			row = this->getUindex(staggeredElement);
			break;
		case DisplacementIndex::V:
			row = this->getVindex(staggeredElement);
			break;
		case DisplacementIndex::W:
			row = this->getWindex(staggeredElement);
			break;
	}
	this->linearSystem.coefficients.reserve(this->linearSystem.coefficients.size() + scalarStencilOnStaggeredElements.size());
	for(auto& keyValuePair: scalarStencilOnStaggeredElements)
	{
		unsigned column;
		switch(displacementComponent)
		{
			case DisplacementIndex::U:
				column = this->getUindex(keyValuePair.first);
				break;
			case DisplacementIndex::V:
				column = this->getVindex(keyValuePair.first);
				break;
			case DisplacementIndex::W:
				column = this->getWindex(keyValuePair.first);
				break;
		}
		this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(row, column, keyValuePair.second) );
	}
	return;
}

void Terzaghi::insertPressureAccumulationTermInIndependent(void)
{
	const double compressibility = this->porosity * this->fluidCompressibility + (this->alpha - this->porosity) * this->solidCompressibility;
	for(auto& element: this->grid.elements)
	{
		const unsigned index = this->getPindex(element);
		this->linearSystem.independent(index) = compressibility * element->getVolume() * this->oldSolution[index];
	}
	return;
}

void Terzaghi::insertPressureVolumeDilatationTermInIndependent(void)
{
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
	{
		double independentValue = this->alpha * (staggeredQuadrangle->getAreaVector().dot(this->getDisplacementVector(staggeredQuadrangle)));
		const unsigned frontRow = getPindex(staggeredQuadrangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
		const unsigned backRow = getPindex(staggeredQuadrangle->elements[1]);
		this->linearSystem.independent[backRow] += independentValue;
	}
	for(auto staggeredTriangle: this->grid.staggeredTriangles)
	{
		double independentValue = this->alpha * (staggeredTriangle->getAreaVector().dot(this->getDisplacementVector(staggeredTriangle)));
		const unsigned frontRow = getPindex(staggeredTriangle->elements[0]);
		this->linearSystem.independent[frontRow] += - independentValue;
	}
	return;
}

Eigen::MatrixXd Terzaghi::getPermutationMatrix(unsigned i, unsigned j)
{
	constexpr unsigned matrixSize = 3;
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

Eigen::MatrixXd Terzaghi::getMechanicalPropertiesMatrix(const unsigned i, const unsigned j)
{
	constexpr unsigned matrixSize = 3;
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

void Terzaghi::setOldPressure(const std::function<double(Eigen::Vector3d)> oldPressureFunction)
{
	for(auto& element: this->grid.elements)
	{
		const unsigned index = getPindex(element);
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
		const unsigned index = getPindex( this->grid.elements[count] );
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
		const unsigned uIndex = getUindex( staggeredElement );
		const unsigned vIndex = getVindex( staggeredElement );
		const unsigned wIndex = getWindex( staggeredElement );
		this->oldSolution[uIndex] = displacements[count].x();
		this->oldSolution[vIndex] = displacements[count].y();
		this->oldSolution[wIndex] = displacements[count].z();
	}
	return;
}