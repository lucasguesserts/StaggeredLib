#include <Grid/Grid2DWithStaggeredElementsExport.hpp>

void Grid2DWithStaggeredElementsExport::cgns(const std::string& fileName, const Grid2DWithStaggeredElements& grid)
{
	GridDataShared gridData = MakeShared<GridData>();
	gridData->dimension = 2;
	Grid2DWithStaggeredElementsExport::exportCoordinates(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportStaggeredTriangles(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportStaggeredQuadrangles(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportLines(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportRegions(gridData);
	CgnsCreator2D cgnsCreator(gridData, fileName);
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
	for(unsigned count=0 ; count<grid.staggeredTriangles.size() ; ++count)
	{
		auto& staggeredTriangle = grid.staggeredTriangles[count];
		auto& connectivity = gridData->triangleConnectivity[count];
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
	for(unsigned count=0 ; count<grid.staggeredQuadrangles.size() ; ++count)
	{
		auto& staggeredQuadrangle = grid.staggeredQuadrangles[count];
		auto& connectivity = gridData->quadrangleConnectivity[count];
		connectivity[0] = staggeredQuadrangle->vertices[0]->getIndex();
		connectivity[1] = numberOfVertices + staggeredQuadrangle->elements[0]->getIndex();
		connectivity[2] = staggeredQuadrangle->vertices[1]->getIndex();
		connectivity[3] = numberOfVertices + staggeredQuadrangle->elements[1]->getIndex();
		connectivity[4] = staggeredQuadrangle->getIndex();
	}
	return;
}

void Grid2DWithStaggeredElementsExport::exportRegions(GridDataShared gridData)
{
	Grid2DWithStaggeredElementsExport::exportQuadrangleRegion(gridData);
	Grid2DWithStaggeredElementsExport::exportTriangleRegion(gridData);
	Grid2DWithStaggeredElementsExport::exportLineRegion(gridData);
}

void Grid2DWithStaggeredElementsExport::exportTriangleRegion(GridDataShared gridData)
{
	const unsigned numberOfTriangles = gridData->triangleConnectivity.size();
	RegionData region;
	region.elementsOnRegion.resize(numberOfTriangles);
	region.name = "triangles";
	for(unsigned count=0 ; count<numberOfTriangles ; ++count)
	{
		region.elementsOnRegion[count] = gridData->triangleConnectivity[count].back();
	}
	gridData->regions.emplace_back(std::move(region));
	return;
}

void Grid2DWithStaggeredElementsExport::exportQuadrangleRegion(GridDataShared gridData)
{
	const unsigned numberOfQuadrangles = gridData->quadrangleConnectivity.size();
	RegionData region;
	region.elementsOnRegion.resize(numberOfQuadrangles);
	region.name = "quadrangles";
	for(unsigned count=0 ; count<numberOfQuadrangles ; ++count)
	{
		region.elementsOnRegion[count] = gridData->quadrangleConnectivity[count].back();
	}
	gridData->regions.emplace_back(std::move(region));
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

void Grid2DWithStaggeredElementsExport::exportLineRegion(GridDataShared gridData)
{
	const unsigned numberOfLines = gridData->lineConnectivity.size();
	RegionData region;
	region.elementsOnRegion.resize(numberOfLines);
	region.name = "lines";
	for(unsigned count=0 ; count<numberOfLines ; ++count)
	{
		region.elementsOnRegion[count] = gridData->lineConnectivity[count].back();
	}
	gridData->regions.emplace_back(std::move(region));
	return;
}