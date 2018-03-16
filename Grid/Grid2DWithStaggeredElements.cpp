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
	const unsigned maxNumberOfStaggeredElements = numberOfTriangles + numberOfQuadrangles;
	this->staggeredElementDefinition.reserve(maxNumberOfStaggeredElements);
	return;
}

void Grid2DWithStaggeredElements::shrinkStaggeredElementDefinition(void)
{
	this->staggeredElementDefinition.shrink_to_fit();
	return;
}
