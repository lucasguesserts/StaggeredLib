#include <Terzaghi/Terzaghi.hpp>

Terzaghi::Terzaghi(const std::string& gridFile)
	: grid(gridFile)
{
	const unsigned linearSystemSize = this->grid.elements.size() + 3 * this->grid.staggeredElements.size();
	this->linearSystem.setSize(linearSystemSize);
	return;
}