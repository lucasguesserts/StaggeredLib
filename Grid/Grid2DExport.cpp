#include <Grid/Grid2DExport.hpp>

void Grid2DExport::cgns(const std::string& fileName, const Grid2D& grid)
{
	GridDataShared gridData = MakeShared<GridData>();
	gridData->dimension = 2;
	Grid2DExport::exportCoordinates(grid, gridData);
	Grid2DExport::exportTriangles(grid, gridData);
	Grid2DExport::exportQuadrangles(grid, gridData);
	Grid2DExport::exportRegion(grid, gridData);
	CgnsCreator2D cgnsCreator(gridData, fileName);
	return;
}

void Grid2DExport::exportCoordinates(const Grid2D& grid, GridDataShared gridData)
{
	const unsigned numberOfVertices = grid.vertices.size();
	gridData->coordinates.resize(numberOfVertices);
	for(auto& vertex: grid.vertices)
	{
		auto& coordinate = gridData->coordinates[vertex.getIndex()];
		coordinate[0] = vertex.x();
		coordinate[1] = vertex.y();
	}
	return;
}

void Grid2DExport::exportTriangles(const Grid2D& grid, GridDataShared gridData)
{
	const unsigned numberOfTriangles = grid.triangles.size();
	gridData->triangleConnectivity.resize(numberOfTriangles);
	for(unsigned count=0 ; count<numberOfTriangles ; ++count)
	{
		auto& triangle = grid.triangles[count];
		auto& connectivity = gridData->triangleConnectivity[count];
		connectivity[0] = triangle.vertices[0]->getIndex();
		connectivity[1] = triangle.vertices[1]->getIndex();
		connectivity[2] = triangle.vertices[2]->getIndex();
		connectivity[3] = triangle.getIndex();
	}
	return;
}

void Grid2DExport::exportQuadrangles(const Grid2D& grid, GridDataShared gridData)
{
	const unsigned numberOfQuadrangles = grid.quadrangles.size();
	gridData->quadrangleConnectivity.resize(numberOfQuadrangles);
	for(unsigned count=0 ; count<numberOfQuadrangles ; ++count)
	{
		auto& quadrangle = grid.quadrangles[count];
		auto& connectivity = gridData->quadrangleConnectivity[count];
		connectivity[0] = quadrangle.vertices[0]->getIndex();
		connectivity[1] = quadrangle.vertices[1]->getIndex();
		connectivity[2] = quadrangle.vertices[2]->getIndex();
		connectivity[3] = quadrangle.vertices[3]->getIndex();
		connectivity[4] = quadrangle.getIndex();
	}
	return;
}

void Grid2DExport::exportRegion(const Grid2D& grid, GridDataShared gridData)
{
	const unsigned numberOfElements = grid.elements.size();
	RegionData region;
	region.name = "elements";
	region.elementsOnRegion.resize(numberOfElements);
	for(unsigned count=0 ; count<numberOfElements ; ++count)
		region.elementsOnRegion[count] = grid.elements[count]->getIndex();
	gridData->regions.emplace_back(std::move(region));
	return;
}

void Grid2DExport::csv(const std::string& fileName, Grid2D& grid)
{
	std::ofstream csvFile;
	csvFile.open(fileName, std::ios::out);
	// X
	for(unsigned count=0 ; count<(grid.elements.size()-1) ; ++count)
	{
		auto centroid = grid.elements[count]->getCentroid();
		csvFile << doubleToString(centroid.x()) << ",";
	}
	{
		auto centroid = grid.elements.back()->getCentroid();
		csvFile << doubleToString(centroid.x());
	}
	csvFile << std::endl;
	// Y
	for(unsigned count=0 ; count<(grid.elements.size()-1) ; ++count)
	{
		auto centroid = grid.elements[count]->getCentroid();
		csvFile << doubleToString(centroid.y()) << ",";
	}
	{
		auto centroid = grid.elements.back()->getCentroid();
		csvFile << doubleToString(centroid.y());
	}
	csvFile << std::endl;
	csvFile.close();
	return;
}

void Grid2DExport::csvAppendTimeSolution(const std::string& fileName, const double timeInstant, const std::vector<double>& solution)
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