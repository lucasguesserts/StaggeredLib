#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <Grid/GridData.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/Grid2DVerticesWithNeighborElements.hpp>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>

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

TestCase("Scalar map", "[ScalarStencil]")
{
	section("operator +")
	{
		section("Equal entries")
		{
			const unsigned index = 5;
			const double scalar[2] = {3.4, -2.3};
			ScalarStencil first = { {index,scalar[0]} };
			ScalarStencil second = { {index,scalar[1]} };
			ScalarStencil sum = first + second;
			check(sum[index]==(scalar[0]+scalar[1]));
		}
		section("Different entries")
		{
			const unsigned index[2] = {1, 8};
			const double scalar[2] = {9.9, -4.7};
			ScalarStencil first = { {index[0],scalar[0]} };
			ScalarStencil second = { {index[1],scalar[1]} };
			ScalarStencil sum = first + second;
			check(sum[index[0]]==scalar[0]);
			check(sum[index[1]]==scalar[1]);
		}
		section("Mixed")
		{
			const unsigned index[3] = {1, 8, 19};
			const double scalar[3] = {9.9, -4.7, 6.3};
			ScalarStencil first = { {index[0],scalar[0]} , {index[1],scalar[1]} };
			ScalarStencil second = { {index[1],scalar[1]}, {index[2],scalar[2]} };
			ScalarStencil sum = first + second;
			check(sum[index[0]]==scalar[0]);
			check(sum[index[1]]==(scalar[1]+scalar[1]));
			check(sum[index[2]]==scalar[2]);
		}
	}
	section("operator*(double scalar, ScalarStencil scalarMap)")
	{
		const unsigned index = 2;
		const double scalarMapEntry = 6.7;
		const double scalarMultiplier = 2.8;
		ScalarStencil scalarMap = { {index,scalarMapEntry} };
		ScalarStencil product = scalarMultiplier * scalarMap;
		check(product[index]==(scalarMultiplier*scalarMapEntry));
	}
	section("+ and * mixed")
	{
		const unsigned index[3] = {2, 7, 4};
		const double value[4] = {5.8, -9.5, -3.2, 6.1};
		ScalarStencil first = {{index[0], value[0]}, {index[1], value[1]}};
		ScalarStencil second = {{index[2], value[2]}, {index[1], value[3]}};
		const double scalar = 0.4;
		ScalarStencil result = scalar*first + second;
		check(result[2]==(scalar*value[0]));
		check(result[4]==(value[2]));
		check(result[7]==(scalar*value[1]+value[3]));
	}
}
