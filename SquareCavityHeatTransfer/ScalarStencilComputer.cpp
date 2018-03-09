#include <SquareCavityHeatTransfer/ScalarStencilComputer.hpp>

ScalarStencil ScalarStencilComputer::inverseDistance(const Vertex& vertex,const std::vector<Element*>& vertexNeighborElements)
{
	ScalarStencil vertexScalarStencil;
	for(Element* element: vertexNeighborElements)
		vertexScalarStencil[element->getIndex()] = 1 / (vertex - element->getCentroid()).norm();
	double inverseDistanceSum = 0.0;
	for(auto& keyValuePair: vertexScalarStencil)
		inverseDistanceSum += keyValuePair.second;
	return (1/inverseDistanceSum) * vertexScalarStencil;
}
