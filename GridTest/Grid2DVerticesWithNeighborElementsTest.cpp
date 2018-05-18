#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <vector>
#include <string>

#include <CGNSFile/CGNSFile.hpp>

#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>

#include <Grid/GridData_2.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>

TestCase("Grid that define vertex with neighbors elements", "[Grid2DVerticesWithNeighborElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	Grid2DVerticesWithNeighborElements grid(cgnsGridFileName);
	section("Vertex 3 neighborhood")
	{
		const unsigned vertexIndex = 3;
		std::vector<Element*>& vertexNeighborhood = grid.verticesNeighborElements[vertexIndex];
		section("Number of neighbor elements")
		{
			const unsigned numberOfNeighborElements = 3;
			check(vertexNeighborhood.size()==numberOfNeighborElements);
		}
		section("Neighbor elements")
		{
			check(contains(vertexNeighborhood, grid.elements[0]));
			check(contains(vertexNeighborhood, grid.elements[2]));
			check(contains(vertexNeighborhood, grid.elements[3]));
		}
	}
}