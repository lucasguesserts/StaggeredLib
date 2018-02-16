#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <Grid/GridData.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/Grid2DVerticesWithNeighborElements.hpp>

TestCase("Analytical solution", "[SquareCavityHeatTransfer]")
{
	const unsigned numberOfPoints = 4;
	Eigen::Matrix<double,4,3> coordinates;
	coordinates << 0.1, 0.1, 0.0,
	               0.5, 0.4, 0.0,
	               0.9, 1.0, 0.0,
	               0.7, 3.0, 0.0;
	Eigen::Vector4d solution, correctSolution;
	correctSolution << 0.008545119856,
	                   0.1397977728,
	                   0.3090169944,
	                   434.0323775;
	solution = SquareCavityHeatTransfer::computeAnalyticalSolution(coordinates);
	for(unsigned pointIndex=0 ; pointIndex<numberOfPoints ; ++pointIndex)
	{
		check(solution[pointIndex]==Approx(correctSolution[pointIndex]));
	}
}

TestCase("Grid that define vertex with neighbors elements", "[Grid2DVerticesWithNeighborElements]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	GridData gridData(cgnsGridFileName);
	Grid2DVerticesWithNeighborElements grid(gridData);
	section("Vertex 3 neighborhood")
	{
		const unsigned vertexIndex = 3;
		std::vector<const Element*>& vertexNeighborhood = grid.verticesNeighborElements[vertexIndex];
		section("Number of neighbor elements")
		{
			const unsigned numberOfNeighborElements = 3;
			check(vertexNeighborhood.size()==numberOfNeighborElements);
		}
		section("Neighbor elements")
		{
			check(contains(vertexNeighborhood, grid.elements[0]));
			check(contains(vertexNeighborhood, grid.elements[1]));
			check(contains(vertexNeighborhood, grid.elements[4]));
		}
	}
}
