#include <Grid/Grid2DWithStaggeredElements.hpp>

Grid2DWithStaggeredElements::Grid2DWithStaggeredElements(const GridData& gridData)
	: Grid2DVerticesWithNeighborElements(gridData)
{
	this->allocateStaggeredElementDefinition(gridData);
	this->shrinkStaggeredElementDefinition();
	return;
}

void Grid2DWithStaggeredElements::allocateStaggeredElementDefinition(const GridData& gridData)
{
	const unsigned numberOfTriangles = gridData.triangle.size();
	const unsigned numberOfQuadrangles = gridData.quadrangle.size();
	const unsigned maxNumberOfStaggeredElements = 3*numberOfQuadrangles + 2*numberOfTriangles + 1;
	this->staggeredElementDefinition.reserve(maxNumberOfStaggeredElements);
	return;
}

void Grid2DWithStaggeredElements::shrinkStaggeredElementDefinition(void)
{
	this->staggeredElementDefinition.shrink_to_fit();
	return;
}

std::tuple<bool,unsigned> Grid2DWithStaggeredElements::findStaggeredElementDefinition(const StaggeredElementDefinition& staggeredElementDefinition)
{
	bool elementExists;
	auto iterator = std::find(this->staggeredElementDefinition.cbegin(), this->staggeredElementDefinition.cend(), staggeredElementDefinition);
	if(iterator==this->staggeredElementDefinition.cend())
		elementExists = false;
	else
		elementExists = true;
	unsigned position = std::distance(this->staggeredElementDefinition.cbegin(),iterator);
	return std::make_tuple(elementExists,position);
}