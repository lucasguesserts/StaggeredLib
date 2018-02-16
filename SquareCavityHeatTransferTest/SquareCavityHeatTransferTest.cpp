#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <Grid/GridData.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/Grid2DVerticesWithNeighborElements.hpp>
#include <SquareCavityHeatTransfer/ScalarMap.hpp>

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

TestCase("Scalar map", "[ScalarMap]")
{
	section("operator +")
	{
		section("Equal entries")
		{
			const unsigned handle = 5;
			const double scalar[2] = {3.4, -2.3};
			ScalarMap first = { {handle,scalar[0]} };
			ScalarMap second = { {handle,scalar[1]} };
			ScalarMap sum = first + second;
			check(sum[handle]==(scalar[0]+scalar[1]));
		}
		section("Different entries")
		{
			const unsigned handle[2] = {1, 8};
			const double scalar[2] = {9.9, -4.7};
			ScalarMap first = { {handle[0],scalar[0]} };
			ScalarMap second = { {handle[1],scalar[1]} };
			ScalarMap sum = first + second;
			check(sum[handle[0]]==scalar[0]);
			check(sum[handle[1]]==scalar[1]);
		}
		section("Mixed")
		{
			const unsigned handle[3] = {1, 8, 19};
			const double scalar[3] = {9.9, -4.7, 6.3};
			ScalarMap first = { {handle[0],scalar[0]} , {handle[1],scalar[1]} };
			ScalarMap second = { {handle[1],scalar[1]}, {handle[2],scalar[2]} };
			ScalarMap sum = first + second;
			check(sum[handle[0]]==scalar[0]);
			check(sum[handle[1]]==(scalar[1]+scalar[1]));
			check(sum[handle[2]]==scalar[2]);
		}
	}
	section("operator*(double scalar, ScalarMap scalarMap)")
	{
		const unsigned handle = 2;
		const double scalarMapEntry = 6.7;
		const double scalarMultiplier = 2.8;
		ScalarMap scalarMap = { {handle,scalarMapEntry} };
		ScalarMap product = scalarMultiplier * scalarMap;
		check(product[handle]==(scalarMultiplier*scalarMapEntry));
	}
}
