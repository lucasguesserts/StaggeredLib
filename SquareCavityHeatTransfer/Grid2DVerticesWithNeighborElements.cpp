#include <SquareCavityHeatTransfer/Grid2DVerticesWithNeighborElements.hpp>
#include <GeometricEntity/Vertex.hpp>

Grid2DVerticesWithNeighborElements::Grid2DVerticesWithNeighborElements(const GridData& gridData)
	: Grid2D(gridData)
{
	this->setVerticesNeighborElements();
}

void Grid2DVerticesWithNeighborElements::setVerticesNeighborElements(void)
{
	const unsigned numberOfVertices = this->vertices.size();
	this->verticesNeighborElements.resize(numberOfVertices);
	for(const Element* element: this->elements)
	{
		for(const Vertex* const vertex: element->vertices)
		{
			const unsigned vertexIndex = vertex->getIndex();
			this->verticesNeighborElements[vertexIndex].push_back(element);
		}
	}
	return;
}
