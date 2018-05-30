#include <Terzaghi/Terzaghi.hpp>
#include <stdexcept>

Terzaghi::Terzaghi(const std::string& gridFile)
	: grid(gridFile)
{
	this->numberOfElements = this->grid.elements.size();
	this->numberOfStaggeredElements = this->grid.staggeredElements.size();
	this->linearSystemSize = this->numberOfElements + 3 * this->numberOfStaggeredElements;
	this->linearSystem.setSize(this->linearSystemSize);
	this->oldSolution.resize(this->linearSystemSize);

	this->initializePressureGradient();
	return;
}

void Terzaghi::initializeScalarStencilOnVertices(void)
{
	this->scalarStencilOnVertices = this->grid.computeScalarStencilOnVertices();
}

void Terzaghi::initializePressureGradient(void)
{
	this->initializeScalarStencilOnVertices();
	this->pressureGradient.resize(this->grid.staggeredElements.size());
	for(auto staggeredQuadrangle: this->grid.staggeredQuadrangles)
		this->pressureGradient[staggeredQuadrangle->getIndex()] =
			this->grid.computeVectorStencilOnQuadrangle(*staggeredQuadrangle,this->scalarStencilOnVertices);
	return;
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

unsigned Terzaghi::getVindex(StaggeredElement2D* staggeredElement)
{
	return this->numberOfElements + this->numberOfStaggeredElements + staggeredElement->getIndex();
}

unsigned Terzaghi::getWindex(StaggeredElement2D* staggeredElement)
{
	return this->numberOfElements + (2 * this->numberOfStaggeredElements) + staggeredElement->getIndex();
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