#include <Grid/Grid.hpp>
#include <stdexcept>

Grid::Grid(const GridData_2& gridData)
{
	this->dimension = gridData.dimension;
	buildVertices_2(gridData);
}

Grid::Grid(const std::string& fileName)
{
	this->readCgnsFile(fileName);
	this->buildVertices();
}

void Grid::readCgnsFile(const std::string& fileName)
{
	if(!CgnsReader::isCgnsFile(fileName))
		throw std::runtime_error("The file '" + fileName + "' is not a valid cgns file.");
	cgnsReader = std::make_unique<CgnsReader2D>(fileName);
	return;
}

void Grid::buildVertices(void)
{
	const std::vector<std::array<double, 3>>& coordinates = this->cgnsReader->gridData->coordinates;
	this->vertices.reserve(coordinates.size());
	unsigned vertexIndex = 0;
	for(auto& coordinate: coordinates)
	{
		this->vertices.push_back(Vertex(coordinate[0], coordinate[1], coordinate[2], vertexIndex));
		++vertexIndex;
	}
	return;
}

void Grid::buildVertices_2(const GridData_2& gridData)
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
