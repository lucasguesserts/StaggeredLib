#include <SquareCavityHeatTransfer/ScalarStencilComputer.hpp>

ScalarStencil ScalarStencilComputer::inverseDistance(const Vertex& vertex,const std::vector<Element*>& vertexNeighborElements)
{
	ScalarStencil vertexScalarStencil;
	for(Element* element: vertexNeighborElements)
		vertexScalarStencil[element->getIndex()] = 1 / (vertex - element->getCentroid()).norm();
	ScalarStencilComputer::normalizeScalarStencil(vertexScalarStencil);
	return vertexScalarStencil;
}

void ScalarStencilComputer::normalizeScalarStencil(ScalarStencil& scalarStencil)
{
	double sum = 0.0;
	for(auto& keyValuePair: scalarStencil)
		sum += keyValuePair.second;
	for(auto& keyValuePair: scalarStencil)
		keyValuePair.second *= (1/sum);
	return;
}

std::vector<ScalarStencil> ScalarStencilComputer::inverseDistance(const Grid2DVerticesWithNeighborElements& grid)
{
	std::vector<ScalarStencil> scalarStencilVector;
	scalarStencilVector.reserve(grid.vertices.size());
	for(const Vertex& vertex: grid.vertices)
		scalarStencilVector.emplace_back( ScalarStencilComputer::inverseDistance(vertex,grid.verticesNeighborElements[vertex.getIndex()]));
	return scalarStencilVector;
}

std::vector<ScalarStencil> ScalarStencilComputer::elements(const Grid2DVerticesWithNeighborElements& grid)
{
	constexpr double weightValue = 1.0;
	std::vector<ScalarStencil> scalarStencilOnElements;
	scalarStencilOnElements.reserve(grid.elements.size());
	for(Element* element: grid.elements)
	{
		ScalarStencil elementScalarStencil = { {element->getIndex(), weightValue} };
		scalarStencilOnElements.emplace_back(elementScalarStencil);
	}
	return scalarStencilOnElements;
}
