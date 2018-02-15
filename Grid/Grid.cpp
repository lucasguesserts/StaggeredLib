#include <Grid/Grid.hpp>

Grid::Grid(const GridData& gridData)
{
	this->dimension = gridData.dimension;
	buildVertices(gridData);
}

void Grid::buildVertices(const GridData& gridData)
{
	const unsigned numberOfVertices = gridData.coordinates.rows();
	this->vertices.reserve(numberOfVertices);
	for(unsigned vertexIndex=0 ; vertexIndex<numberOfVertices ; ++vertexIndex)
	{
		const double x = gridData.coordinates(vertexIndex,0);
		const double y = gridData.coordinates(vertexIndex,1);
		const double z = gridData.coordinates(vertexIndex,2);
		this->vertices.push_back(Vertex(x, y, z, vertexIndex));
	}
	return ;
}
