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
