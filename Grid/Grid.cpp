#include <Grid/Grid.hpp>
#include <stdexcept>

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