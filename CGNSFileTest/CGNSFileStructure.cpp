#include <Utils/Test.hpp>
#include <vector>
#include <string>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>

const std::string gridDirectory = GRID_DIRECTORY;

TestCase("CGNS file structure basic - coordinates and element connectivity", "[CGNSFile]")
{
	const std::string cgnsGridFileName = gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	section("read coordinates X")
	{
		std::vector<double> verticesCoordinateX{0.0, 1.5, 3.0, 0.0, 1.5, 3.0, 0.0, 1.5, 3.0};
		std::vector<double> readCoordinateX = cgnsFile.readCoordinate("CoordinateX");
		check(readCoordinateX==verticesCoordinateX);
	}
	section("read coordinates Y")
	{
		std::vector<double> verticesCoordinateY{0.0, 0.0, 0.0, 1.5, 1.5, 1.5, 3.0, 3.0, 3.0};
		std::vector<double> readCoordinateY = cgnsFile.readCoordinate("CoordinateY");
		check(readCoordinateY==verticesCoordinateY);
	}
	section("read quadrangles")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		std::vector< ElementDefinition<4> > quadrangle(numberOfQuadrangles);
			quadrangle[0].index = 0;
			quadrangle[0].connectivity << 0, 1, 4, 3;
			quadrangle[1].index = 1;
			quadrangle[1].connectivity << 1, 2, 5, 4;
		std::vector< ElementDefinition<4> > readQuadrangle = cgnsFile.readElementsDefinition<4,cgns::QUAD_4>();
		check(readQuadrangle==quadrangle);
	}
	section("read triangles")
	{
		constexpr unsigned numberOfTriangles = 4;
		std::vector< ElementDefinition<3> > triangle(numberOfTriangles);
			triangle[0].index = 2;
			triangle[0].connectivity << 3, 4, 7;
			triangle[1].index = 3;
			triangle[1].connectivity << 3, 7, 6;
			triangle[2].index = 4;
			triangle[2].connectivity << 4, 5, 8;
			triangle[3].index = 5;
			triangle[3].connectivity << 4, 8, 7;
		std::vector< ElementDefinition<3> > readTriangle = cgnsFile.readElementsDefinition<3,cgns::TRI_3>();
		check(readTriangle==triangle);
	}
	section("read lines")
	{
		constexpr unsigned numberOfLines = 8;
		std::vector< ElementDefinition<2> > line(numberOfLines);
			line[0].index = 6;
			line[0].connectivity << 1, 0;
			line[1].index = 7;
			line[1].connectivity << 2, 1;
			line[2].index = 8;
			line[2].connectivity << 6, 7;
			line[3].index = 9;
			line[3].connectivity << 7, 8;
			line[4].index = 10;
			line[4].connectivity << 3, 0;
			line[5].index = 11;
			line[5].connectivity << 6, 3;
			line[6].index = 12;
			line[6].connectivity << 5, 2;
			line[7].index = 13;
			line[7].connectivity << 8, 5;
		std::vector< ElementDefinition<2> > readLine = cgnsFile.readElementsDefinition<2,cgns::BAR_2>();
		check(readLine==line);
	}
}