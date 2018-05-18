#include <Grid/Grid2DWithStaggeredElementsExport.hpp>

void Grid2DWithStaggeredElementsExport::cgns(const std::string& fileName, const Grid2DWithStaggeredElements& grid)
{
	GridDataShared gridData = MakeShared<GridData>();
	gridData->dimension = 2;
	Grid2DWithStaggeredElementsExport::exportCoordinates(grid, gridData);
	// Grid2DWithStaggeredElementsExport::exportStaggeredTriangles(grid, gridData);
	// Grid2DWithStaggeredElementsExport::exportStaggeredQuadrangles(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportLines(grid, gridData);
	CgnsCreator2D(gridData, fileName);
	return;
}

void Grid2DWithStaggeredElementsExport::exportCoordinates(const Grid2DWithStaggeredElements& grid, GridDataShared gridData)
{
	const unsigned numberOfElements = grid.elements.size();
	const unsigned numberOfVertices = grid.vertices.size();
	gridData->coordinates.resize(numberOfVertices + numberOfElements);
	for(auto& vertex: grid.vertices)
	{
		auto& coordinate = gridData->coordinates[vertex.getIndex()];
		coordinate[0] = vertex.x();
		coordinate[1] = vertex.y();
	}
	for(auto element: grid.elements)
	{
		auto& coordinate = gridData->coordinates[numberOfVertices + element->getIndex()];
		auto centroid = element->getCentroid();
		coordinate[0] = centroid.x();
		coordinate[1] = centroid.y();
	}
	return;
}

void Grid2DWithStaggeredElementsExport::exportStaggeredTriangles(const Grid2DWithStaggeredElements& grid, GridDataShared gridData)
{
	const unsigned numberOfVertices = grid.vertices.size();
	gridData->triangleConnectivity.resize(grid.staggeredTriangles.size());
	for(auto staggeredTriangle: grid.staggeredTriangles)
	{
		auto& connectivity = gridData->triangleConnectivity[staggeredTriangle->getIndex()];
		connectivity[0] = staggeredTriangle->vertices[0]->getIndex();
		connectivity[1] = numberOfVertices + staggeredTriangle->elements[0]->getIndex();
		connectivity[2] = staggeredTriangle->vertices[1]->getIndex();
		connectivity[3] = staggeredTriangle->getIndex();
	}
	return;
}

void Grid2DWithStaggeredElementsExport::exportStaggeredQuadrangles(const Grid2DWithStaggeredElements& grid, GridDataShared gridData)
{
	const unsigned numberOfVertices = grid.vertices.size();
	gridData->quadrangleConnectivity.resize(grid.staggeredQuadrangles.size());
	for(auto staggeredQuadrangle: grid.staggeredQuadrangles)
	{
		auto& connectivity = gridData->quadrangleConnectivity[staggeredQuadrangle->getIndex()];
		connectivity[0] = staggeredQuadrangle->vertices[0]->getIndex();
		connectivity[1] = numberOfVertices + staggeredQuadrangle->elements[0]->getIndex();
		connectivity[2] = staggeredQuadrangle->vertices[1]->getIndex();
		connectivity[3] = numberOfVertices + staggeredQuadrangle->elements[1]->getIndex();
		connectivity[4] = staggeredQuadrangle->getIndex();
	}
	return;
}

void Grid2DWithStaggeredElementsExport::exportLines(const Grid2DWithStaggeredElements& grid, GridDataShared gridData)
{
	gridData->lineConnectivity.resize(grid.lines.size());
	unsigned index = 0u;
	for(auto& line: grid.lines)
	{
		auto& connectivity = gridData->lineConnectivity[index];
		connectivity[0] = line.vertices[0]->getIndex();
		connectivity[1] = line.vertices[1]->getIndex();
		connectivity[2] = line.getIndex();
		++index;
	}
	return;
}