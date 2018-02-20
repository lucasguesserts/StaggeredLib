#include <array>

#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <Grid/GridData.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/Grid2DVerticesWithNeighborElements.hpp>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <SquareCavityHeatTransfer/VectorStencil.hpp>

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

TestCase("Scalar stencil", "[ScalarStencil]")
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

TestCase("Vector stencil", "[VectorStencil]")
{
	section("ScalarStencil * Eigen::Vector3d")
	{
		const double value[2] = {7.4, -8.3};
		const unsigned index[2] = {2, 9};
		ScalarStencil scalarStencil = { {index[0],value[0]}, {index[1],value[1]} };
		const Eigen::Vector3d vector(-6.3, 1.2, 4.5);
		VectorStencil result = scalarStencil*vector;
		check(result[index[0]]==(value[0]*vector));
		check(result[index[1]]==(value[1]*vector));
	}
	section("VectorStencil + VectorStencil")
	{
		const Eigen::Vector3d vector[4] = {{2.7,-1.8,4.3}, {-12.9,0.8,2.2}, {18.7,3.9,8.74}, {7.9,-8.3,-0.9}};
		const unsigned index[3] = {3, 27, 5};
		VectorStencil first = {{index[0],vector[0]}, {index[1],vector[1]}};
		VectorStencil second = {{index[1],vector[2]}, {index[2],vector[3]}};
		VectorStencil result = first + second;
		check(result[index[0]]==vector[0]);
		check(result[index[1]]==(vector[1]+vector[2]));
		check(result[index[2]]==vector[3]);
	}
	section("Eigen::Vector3d * VectorStencil")
	{
		const Eigen::Vector3d vector(-6.3, 1.2, 4.5);
		const Eigen::Vector3d vec[2] = {{2.7,-1.8,4.3}, {-12.9,0.8,2.2}};
		const unsigned index[2] = {3, 27};
		VectorStencil vectorStencil = {{index[0],vec[0]}, {index[1],vec[1]}};
		ScalarStencil result = vector * vectorStencil;
		check(result[index[0]]==vector.dot(vec[0]));
		check(result[index[1]]==vector.dot(vec[1]));
	}
	section("double * VectorStencil")
	{
		const double scalar = 2.23;
		const unsigned index = 554;
		const Eigen::Vector3d vector = {-1.6, 8.3, 2.0};
		const VectorStencil vectorStencil = {{index,vector}};
		VectorStencil result = scalar * vectorStencil;
		check(result[index]==(scalar*vector));
	}
}

TestCase("ScalarStencil and scalar field used to interpolate a value", "[ScalarStencil]")
{
	const unsigned numberOfValues = 8;
	Eigen::VectorXd scalarField;
	scalarField.resize(numberOfValues);
	scalarField << 8.1, 2.6, 6.7, 4.5, 1.8, 7.9, 1.3, 2.6;
	section("interpolation with complete ScalarStencil")
	{
		const unsigned scalarStencilSize = 8;
		const std::array<unsigned,scalarStencilSize> key = {0, 1, 2, 3, 4, 5, 6, 7};
		const std::array<double,scalarStencilSize> value = {0.2, 0.4, 0.3, 0.8, 0.1, 0.7, 0.4, 0.2};
		ScalarStencil scalarStencil;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			scalarStencil[key[i]] = value[i];
		double correctInterpolatedValue = 0.0;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			interpolatedValue += scalarStencil[key[i]] * scalarField[key[i]];
		check(correctInterpolatedValue==(scalarStencil*scalarField));
	}
	section("interpolation with partial ScalarStencil")
	{
		const unsigned scalarStencilSize = 4;
		const std::array<unsigned,scalarStencilSize> key = {0, 2, 3, 5};
		const std::array<double,scalarStencilSize> value = {0.2, 0.3, 0.8, 0.7};
		ScalarStencil scalarStencil;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			scalarStencil[key[i]] = value[i];
		double correctInterpolatedValue = 0.0;
		for(unsigned i=0 ; i<scalarStencilSize ; ++i)
			interpolatedValue += scalarStencil[key[i]] * scalarField[key[i]];
		check(correctInterpolatedValue==(scalarStencil*scalarField));
	}
}
