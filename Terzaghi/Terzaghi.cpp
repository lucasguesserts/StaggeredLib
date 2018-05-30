#include <Terzaghi/Terzaghi.hpp>

Terzaghi::Terzaghi(const std::string& gridFile)
	: grid(gridFile)
{
	this->numberOfElements = this->grid.elements.size();
	this->numberOfStaggeredElements = this->grid.staggeredElements.size();
	const unsigned linearSystemSize = this->numberOfElements + 3 * this->numberOfStaggeredElements;
	this->linearSystem.setSize(linearSystemSize);
	return;
}

unsigned Terzaghi::getPindex(Element* element)
{
	return element->getIndex();
}

void Terzaghi::insertPressureAccumulationTermToMatrix(void)
{
	const double compressibility = this->porosity * this->fluidCompressibility + (this->alpha - this->porosity) * this->solidCompressibility;
	for(auto element: this->grid.elements)
	{
		const unsigned index = this->getPindex(element);
		this->linearSystem.coefficients.emplace_back( Eigen::Triplet<double,unsigned>(index, index, compressibility*element->getVolume()) );
	}
	return;
}