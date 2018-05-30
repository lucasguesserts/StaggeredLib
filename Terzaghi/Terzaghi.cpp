#include <Terzaghi/Terzaghi.hpp>
#include <stdexcept>

Terzaghi::Terzaghi(const std::string& gridFile)
	: grid(gridFile)
{
	this->numberOfElements = this->grid.elements.size();
	this->numberOfStaggeredElements = this->grid.staggeredElements.size();
	const unsigned linearSystemSize = this->numberOfElements + 3 * this->numberOfStaggeredElements;
	this->linearSystem.setSize(linearSystemSize);
	this->oldSolution.resize(linearSystemSize);

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