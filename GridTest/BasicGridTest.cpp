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

TestCase("Grid constructor", "[Grid]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	Grid grid(cgnsGridFileName);
	section("vertices")
	{
		const unsigned numberOfVertices = 9;
		std::vector<Vertex> vertices;
		vertices.push_back(Vertex(0.0, 0.0, 0.0, 0));
		vertices.push_back(Vertex(1.5, 0.0, 0.0, 1));
		vertices.push_back(Vertex(3.0, 0.0, 0.0, 2));
		vertices.push_back(Vertex(0.0, 1.5, 0.0, 3));
		vertices.push_back(Vertex(1.5, 1.5, 0.0, 4));
		vertices.push_back(Vertex(3.0, 1.5, 0.0, 5));
		vertices.push_back(Vertex(0.0, 3.0, 0.0, 6));
		vertices.push_back(Vertex(1.5, 3.0, 0.0, 7));
		vertices.push_back(Vertex(3.0, 3.0, 0.0, 8));
		check(grid.vertices==vertices);
	}
}