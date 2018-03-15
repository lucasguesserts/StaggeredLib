#include <Grid/Grid2DInverseDistanceStencil.hpp>

Grid2DInverseDistanceStencil::Grid2DInverseDistanceStencil(GridData& gridData)
	: Grid2DWithStaggeredElements(gridData)
{}

ScalarStencil Grid2DInverseDistanceStencil::computeScalarStencil(Vertex& vertex)
{
	ScalarStencil vertexScalarStencil;
	for(Element* element: this->verticesNeighborElements[vertex.getIndex()])
		vertexScalarStencil[element->getIndex()] = 1 / (vertex - element->getCentroid()).norm();
	Grid2DInverseDistanceStencil::normalizeScalarStencil(vertexScalarStencil);
	return vertexScalarStencil;
}

void Grid2DInverseDistanceStencil::normalizeScalarStencil(ScalarStencil& scalarStencil)
{
	double sum = 0.0;
	for(auto& keyValuePair: scalarStencil)
		sum += keyValuePair.second;
	for(auto& keyValuePair: scalarStencil)
		keyValuePair.second *= (1/sum);
	return;
}
