#include <Grid/GridData.hpp>

void GridData::readCoordinates(CGNSFile& cgnsFile)
{
	this->coordinates.resize(cgnsFile.numberOfVertices,Eigen::NoChange);
	std::vector<double> coordinatesX = cgnsFile.readCoordinate("CoordinateX");
	std::vector<double> coordinatesY = cgnsFile.readCoordinate("CoordinateY");
	for(unsigned vertexIndex=0 ; vertexIndex<cgnsFile.numberOfVertices ; ++vertexIndex)
	{
		this->coordinates(vertexIndex,0) = coordinatesX[vertexIndex];
		this->coordinates(vertexIndex,1) = coordinatesY[vertexIndex];
		this->coordinates(vertexIndex,2) = 0.0;
	}
	return;
}

void GridData::readElementConnectivity(CGNSFile& cgnsFile)
{
	this->quadrangle = cgnsFile.readElementsDefinition<4,cgns::QUAD_4>();
	this->triangle = cgnsFile.readElementsDefinition<3,cgns::TRI_3>();
	this->line = cgnsFile.readElementsDefinition<2,cgns::BAR_2>();
	return;
}

GridData::GridData(CGNSFile& cgnsFile)
{
	this->dimension = cgnsFile.physicalDimension;
	this->readCoordinates(cgnsFile);
	this->readElementConnectivity(cgnsFile);
	return;
}
