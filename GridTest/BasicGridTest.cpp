#include <Utils/Test.hpp>

#include <Grid/GridData_2.hpp>
#include <Grid/Grid.hpp>

TestCase("Grid structure", "[Grid]")
{
	const unsigned dimension = 2;
	const unsigned numberOfVertices = 4;
	GridData_2 gridData;
	gridData.dimension = dimension;
	gridData.coordinates.resize(numberOfVertices,Eigen::NoChange);
	gridData.coordinates <<
		2.0, -5.0,  3.0,
		4.0, -1.0,  6.0,
		4.0,  3.0,  3.4,
		3.0,  7.0, -2.0;
	Grid grid(gridData);
	check(grid.vertices.size()==numberOfVertices);
	check(grid.dimension==dimension);
	for(unsigned vertexIndex=0 ; vertexIndex<numberOfVertices ; ++vertexIndex)
	{
		check(grid.vertices[vertexIndex].getIndex()==vertexIndex);
		for(unsigned entry=0 ; entry<3 ; ++entry)
			check(grid.vertices[vertexIndex](entry)==gridData.coordinates(vertexIndex,entry));
	}
}