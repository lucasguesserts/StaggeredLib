#include <Utils/Test.hpp>
#include <array>
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
		std::array<ElementDefinition<4>,numberOfQuadrangles> quadrangleDefinition;
		// Two answer quadrangles
		quadrangleDefinition[0].index = 12;
		quadrangleDefinition[0].connectivity << 0, 1, 3, 2;
		quadrangleDefinition[1].index = 17;
		quadrangleDefinition[1].connectivity << 3, 5, 6, 4;
		// Two quadrangles in GridData to test
		gridData.quadrangle.resize(numberOfQuadrangles);
		gridData.quadrangle[0].index = 12;
		gridData.quadrangle[0].connectivity << 0, 1, 3, 2;
		gridData.quadrangle[1].index = 17;
		gridData.quadrangle[1].connectivity << 3, 5, 6, 4;
		for(unsigned quadrangleCount=0 ; quadrangleCount<numberOfQuadrangles ; ++quadrangleCount)
			check(gridData.quadrangle[quadrangleCount]==quadrangleDefinition[quadrangleCount]);
	}
	section("triangle connectivity")
	{
		constexpr unsigned numberOfTriangles = 2;
		std::array<ElementDefinition<3>,numberOfTriangles> triangleDefinition;
		// Two answer triangles
		triangleDefinition[0].index = 48;
		triangleDefinition[0].connectivity << 5, 7, 1;
		triangleDefinition[1].index = 94;
		triangleDefinition[1].connectivity << 8, 1, 6;
		// Two triangles in GridData to test
		gridData.triangle.resize(numberOfTriangles);
		gridData.triangle[0].index = 48;
		gridData.triangle[0].connectivity << 5, 7, 1;
		gridData.triangle[1].index = 94;
		gridData.triangle[1].connectivity << 8, 1, 6;
		for(unsigned triangleCount=0 ; triangleCount<numberOfTriangles ; ++triangleCount)
			check(gridData.triangle[triangleCount]==triangleDefinition[triangleCount]);
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
		constexpr unsigned verticesPerQuadrangle = 4;
		std::array<ElementDefinition<4>,numberOfQuadrangles> quadrangleDefinition;
		quadrangleDefinition[0].index = 0;
		quadrangleDefinition[0].connectivity << 0, 1, 4, 3;
		quadrangleDefinition[1].index = 1;
		quadrangleDefinition[1].connectivity << 1, 2, 5, 4;
		for(unsigned count=0 ; count<numberOfQuadrangles ; ++count)
			check(gridData.quadrangle[count]==quadrangleDefinition[count]);
	}
	section("triangle connectivity")
	{
		constexpr unsigned numberOfTriangles = 4;
		constexpr unsigned verticesPerTriangle = 3;
		std::array<ElementDefinition<3>,numberOfTriangles> triangleDefinition;
		triangleDefinition[0].index = 2;
		triangleDefinition[0].connectivity << 3, 4, 7;
		triangleDefinition[1].index = 3;
		triangleDefinition[1].connectivity << 3, 7, 6;
		triangleDefinition[2].index = 4;
		triangleDefinition[2].connectivity << 4, 5, 8;
		triangleDefinition[3].index = 5;
		triangleDefinition[3].connectivity << 4, 8, 7;
		for(unsigned count=0 ; count<numberOfTriangles ; ++count)
			check(gridData.triangle[count]==triangleDefinition[count]);
	}
}