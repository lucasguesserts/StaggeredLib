#include <Utils/Test.hpp>
#include <array>
#include <vector>
#include <string>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>
#include <Grid/GridData.hpp>

TestCase("Grid data structure", "[Grid][GridData][ElementDefinition]")
{
	const unsigned numberOfVertices = 7;
	GridData gridData;
	gridData.coordinates.resize(numberOfVertices,Eigen::NoChange);
	gridData.coordinates <<
		0.0, 0.0, 0.0,
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		1.0, 1.0, 0.0,
		0.5, 2.0, 0.0,
		2.0, 1.5, 0.0,
		1.5, 2.5, 0.0;
	section("dimension")
	{
		const unsigned dimension = 2;
		gridData.dimension = dimension;
		check(gridData.dimension==dimension);
	}
	section("quadrangle connectivity")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		std::vector<ElementDefinition<4>> quadrangleDefinitions(numberOfQuadrangles);
		// Two answer quadrangles
		quadrangleDefinitions[0].index = 12;
		quadrangleDefinitions[0].connectivity << 0, 1, 3, 2;
		quadrangleDefinitions[1].index = 17;
		quadrangleDefinitions[1].connectivity << 3, 5, 6, 4;
		// Two quadrangles in GridData to test
		gridData.quadrangle.resize(numberOfQuadrangles);
		gridData.quadrangle[0].index = 12;
		gridData.quadrangle[0].connectivity << 0, 1, 3, 2;
		gridData.quadrangle[1].index = 17;
		gridData.quadrangle[1].connectivity << 3, 5, 6, 4;
		check(gridData.quadrangle==quadrangleDefinitions);
	}
	section("triangle connectivity")
	{
		constexpr unsigned numberOfTriangles = 2;
		std::vector<ElementDefinition<3>> triangleDefinitions(numberOfTriangles);
		// Two answer triangles
		triangleDefinitions[0].index = 48;
		triangleDefinitions[0].connectivity << 5, 7, 1;
		triangleDefinitions[1].index = 94;
		triangleDefinitions[1].connectivity << 8, 1, 6;
		// Two triangles in GridData to test
		gridData.triangle.resize(numberOfTriangles);
		gridData.triangle[0].index = 48;
		gridData.triangle[0].connectivity << 5, 7, 1;
		gridData.triangle[1].index = 94;
		gridData.triangle[1].connectivity << 8, 1, 6;
		check(gridData.triangle==triangleDefinitions);
	}
	section("line connectivity")
	{
		constexpr unsigned numberOfLines = 2;
		std::vector<ElementDefinition<2>> lineDefinitions(numberOfLines);
		// Two answer lines
		lineDefinitions[0].index = 48;
		lineDefinitions[0].connectivity << 5, 7;
		lineDefinitions[1].index = 94;
		lineDefinitions[1].connectivity << 8, 1;
		// Two lines in GridData to test
		gridData.line.resize(numberOfLines);
		gridData.line[0].index = 48;
		gridData.line[0].connectivity << 5, 7;
		gridData.line[1].index = 94;
		gridData.line[1].connectivity << 8, 1;
		check(gridData.line==lineDefinitions);
	}
}

TestCase("grid reader from CGNS", "[GridData][CGNS]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	section("dimension")
	{
		const unsigned dimension = 2;
		check(gridData.dimension==dimension);
	}
	section("coordinates")
	{
		const unsigned numberOfVertices = 9;
		Eigen::MatrixXd verticesCoordinates;
		verticesCoordinates.resize(numberOfVertices,3);
		verticesCoordinates << 0.0, 0.0, 0.0,
							   1.5, 0.0, 0.0,
							   3.0, 0.0, 0.0,
							   0.0, 1.5, 0.0,
							   1.5, 1.5, 0.0,
							   3.0, 1.5, 0.0,
							   0.0, 3.0, 0.0,
							   1.5, 3.0, 0.0,
							   3.0, 3.0, 0.0;
		check(gridData.coordinates==verticesCoordinates);
	}
	section("quadrangle connectivity")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		std::vector<ElementDefinition<4>> quadrangleDefinitions(numberOfQuadrangles);
		quadrangleDefinitions[0].index = 0;
		quadrangleDefinitions[0].connectivity << 0, 1, 4, 3;
		quadrangleDefinitions[1].index = 1;
		quadrangleDefinitions[1].connectivity << 1, 2, 5, 4;
		check(gridData.quadrangle==quadrangleDefinitions);
	}
	section("triangle connectivity")
	{
		constexpr unsigned numberOfTriangles = 4;
		std::vector<ElementDefinition<3>> triangleDefinitions(numberOfTriangles);
		triangleDefinitions[0].index = 2;
		triangleDefinitions[0].connectivity << 3, 4, 7;
		triangleDefinitions[1].index = 3;
		triangleDefinitions[1].connectivity << 3, 7, 6;
		triangleDefinitions[2].index = 4;
		triangleDefinitions[2].connectivity << 4, 5, 8;
		triangleDefinitions[3].index = 5;
		triangleDefinitions[3].connectivity << 4, 8, 7;
		check(gridData.triangle==triangleDefinitions);
	}
	section("line connectivity")
	{
		constexpr unsigned numberOfLines = 8;
		std::vector<ElementDefinition<2>> lineDefinitions(numberOfLines);
		lineDefinitions[0].index = 6;
		lineDefinitions[0].connectivity << 1, 0;
		lineDefinitions[1].index = 7;
		lineDefinitions[1].connectivity << 2, 1;
		lineDefinitions[2].index = 8;
		lineDefinitions[2].connectivity << 6, 7;
		lineDefinitions[3].index = 9;
		lineDefinitions[3].connectivity << 7, 8;
		lineDefinitions[4].index = 10;
		lineDefinitions[4].connectivity << 3, 0;
		lineDefinitions[5].index = 11;
		lineDefinitions[5].connectivity << 6, 3;
		lineDefinitions[6].index = 12;
		lineDefinitions[6].connectivity << 5, 2;
		lineDefinitions[7].index = 13;
		lineDefinitions[7].connectivity << 8, 5;
		check(gridData.line==lineDefinitions);
	}
}