#include <Grid/Grid2DWithStaggeredElementsExport.hpp>

void Grid2DWithStaggeredElementsExport::cgns(const std::string& fileName, const Grid2DWithStaggeredElements& grid)
{
	GridDataShared gridData = MakeShared<GridData>();
	gridData->dimension = 2;
	Grid2DWithStaggeredElementsExport::exportCoordinates(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportStaggeredTriangles(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportStaggeredQuadrangles(grid, gridData);
	Grid2DWithStaggeredElementsExport::exportRegion(grid, gridData);
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

void Grid2DWithStaggeredElementsExport::exportRegion(const Grid2DWithStaggeredElements& grid, GridDataShared gridData)
{
	const unsigned numberOfStaggeredElements = grid.staggeredElements.size();
	RegionData region;
	region.name = "elements";
	region.elementsOnRegion.resize(numberOfStaggeredElements);
	for(unsigned count=0 ; count<numberOfStaggeredElements ; ++count)
		region.elementsOnRegion[count] = grid.staggeredElements[count].getIndex();
	gridData->regions.emplace_back(std::move(region));
	return;
}

void Grid2DWithStaggeredElementsExport::csv(const std::string& fileName, Grid2DWithStaggeredElements& grid)
{
	std::ofstream csvFile;
	csvFile.open(fileName, std::ios::out);
	// X
	for(unsigned count=0 ; count<(grid.staggeredElements.size()-1) ; ++count)
	{
		auto centroid = grid.staggeredElements[count].getCentroid();
		csvFile << doubleToString(centroid.x()) << ",";
	}
	{
		auto centroid = grid.staggeredElements.back().getCentroid();
		csvFile << doubleToString(centroid.x());
	}
	csvFile << std::endl;
	// Y
	for(unsigned count=0 ; count<(grid.staggeredElements.size()-1) ; ++count)
	{
		auto centroid = grid.staggeredElements[count].getCentroid();
		csvFile << doubleToString(centroid.y()) << ",";
	}
	{
		auto centroid = grid.staggeredElements.back().getCentroid();
		csvFile << doubleToString(centroid.y());
	}
	csvFile << std::endl;
	csvFile.close();
	return;
}

void Grid2DWithStaggeredElementsExport::csvAppendTimeSolution(const std::string& fileName, const double timeInstant, const std::vector<double>& solution)
{
	std::ofstream csvFile;
	csvFile.open(fileName, std::ios::app);
	csvFile << doubleToString(timeInstant) << ",";
	for(unsigned count=0 ; count<(solution.size()-1) ; ++count)
	{
		csvFile << doubleToString(solution[count]) << ",";
	}
	{
		csvFile << doubleToString(solution.back());
	}
	csvFile << std::endl;
	csvFile.close();
	return;
}