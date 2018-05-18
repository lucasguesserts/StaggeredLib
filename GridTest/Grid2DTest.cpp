#include <Utils/Test.hpp>
#include <array>
#include <vector>
#include <string>

#include <CGNSFile/CGNSFile.hpp>
#include <GeometricEntity/Vertex.hpp>

#include <Grid/GridData_2.hpp>
#include <Grid/Grid2D.hpp>

TestCase("Grid 2D build", "[Grid][Grid2D]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	Grid2D grid2D(cgnsGridFileName);
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
		check(grid2D.vertices==vertices);
	}
	section("Elements index")
	{
		for(unsigned elementIndex=0 ; elementIndex<grid2D.elements.size() ; ++elementIndex)
			check(grid2D.elements[elementIndex]->getIndex()==elementIndex);
	}
	section("lines")
	{
		constexpr unsigned numberOfLines = 8;
		section("lines elements centroid")
		{
			std::array<Eigen::Vector3d,numberOfLines> lineCentroid;
			lineCentroid[0] << 0.75, 0.0, 0.0;
			lineCentroid[1] << 2.25, 0.0, 0.0;
			lineCentroid[2] << 0.75, 3.0, 0.0;
			lineCentroid[3] << 2.25, 3.0, 0.0;
			lineCentroid[4] << 0.0, 0.75, 0.0;
			lineCentroid[5] << 0.0, 2.25, 0.0;
			lineCentroid[6] << 3.0, 0.75, 0.0;
			lineCentroid[7] << 3.0, 2.25, 0.0;
			check(grid2D.lines.size()==numberOfLines);
			for(unsigned lineIndex=0 ; lineIndex<numberOfLines ; ++lineIndex)
				check(grid2D.lines[lineIndex].getCentroid()==lineCentroid[lineIndex]);
		}
		section("elements vertices")
		{
			// 0
			check(grid2D.lines[0].vertices[0]==&(grid2D.vertices[1]));
			check(grid2D.lines[0].vertices[1]==&(grid2D.vertices[0]));
			// 1
			check(grid2D.lines[1].vertices[0]==&(grid2D.vertices[2]));
			check(grid2D.lines[1].vertices[1]==&(grid2D.vertices[1]));
			// 2
			check(grid2D.lines[2].vertices[0]==&(grid2D.vertices[6]));
			check(grid2D.lines[2].vertices[1]==&(grid2D.vertices[7]));
			// 3
			check(grid2D.lines[3].vertices[0]==&(grid2D.vertices[7]));
			check(grid2D.lines[3].vertices[1]==&(grid2D.vertices[8]));
			// 4
			check(grid2D.lines[4].vertices[0]==&(grid2D.vertices[3]));
			check(grid2D.lines[4].vertices[1]==&(grid2D.vertices[0]));
			// 5
			check(grid2D.lines[5].vertices[0]==&(grid2D.vertices[6]));
			check(grid2D.lines[5].vertices[1]==&(grid2D.vertices[3]));
			// 6
			check(grid2D.lines[6].vertices[0]==&(grid2D.vertices[5]));
			check(grid2D.lines[6].vertices[1]==&(grid2D.vertices[2]));
			// 7
			check(grid2D.lines[7].vertices[0]==&(grid2D.vertices[8]));
			check(grid2D.lines[7].vertices[1]==&(grid2D.vertices[5]));
		}
	}
	section("triangles")
	{
		constexpr unsigned numberOfTriangles = 4;
		section("triangular elements centroid")
		{
			constexpr unsigned trianglesOffset = 2;
			std::array<Eigen::Vector3d,4> triangleCentroid;
			triangleCentroid[0] << 1.0, 2.0, 0.0;
			triangleCentroid[1] << 0.5, 2.5, 0.0;
			triangleCentroid[2] << 2.5, 2.0, 0.0;
			triangleCentroid[3] << 2.0, 2.5, 0.0;
			for(unsigned elementIndex=0 ; elementIndex<numberOfTriangles ; ++elementIndex)
				check(grid2D.elements[elementIndex+trianglesOffset]->getCentroid()==triangleCentroid[elementIndex]);
		}
		section("elements vertices")
		{
			// 2
			check(grid2D.elements[2]->vertices[0]==&(grid2D.vertices[3]));
			check(grid2D.elements[2]->vertices[1]==&(grid2D.vertices[4]));
			check(grid2D.elements[2]->vertices[2]==&(grid2D.vertices[7]));
			// 3
			check(grid2D.elements[3]->vertices[0]==&(grid2D.vertices[3]));
			check(grid2D.elements[3]->vertices[1]==&(grid2D.vertices[7]));
			check(grid2D.elements[3]->vertices[2]==&(grid2D.vertices[6]));
			// 4
			check(grid2D.elements[4]->vertices[0]==&(grid2D.vertices[4]));
			check(grid2D.elements[4]->vertices[1]==&(grid2D.vertices[5]));
			check(grid2D.elements[4]->vertices[2]==&(grid2D.vertices[8]));
			// 5
			check(grid2D.elements[5]->vertices[0]==&(grid2D.vertices[4]));
			check(grid2D.elements[5]->vertices[1]==&(grid2D.vertices[8]));
			check(grid2D.elements[5]->vertices[2]==&(grid2D.vertices[7]));
		}
	}
	section("quadrangles")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		section("quadrangular elements centroid")
		{
			Eigen::Vector3d quadrangleCentroid[numberOfQuadrangles];
			quadrangleCentroid[0] << 0.75, 0.75, 0.0;
			quadrangleCentroid[1] << 2.25, 0.75, 0.0;
			for(unsigned elementIndex=0 ; elementIndex<numberOfQuadrangles ; ++elementIndex)
				check(grid2D.elements[elementIndex]->getCentroid()==quadrangleCentroid[elementIndex]);
		}
		section("elements vertices")
		{
			// 0
			check(grid2D.elements[0]->vertices[0]==&(grid2D.vertices[0]));
			check(grid2D.elements[0]->vertices[1]==&(grid2D.vertices[1]));
			check(grid2D.elements[0]->vertices[2]==&(grid2D.vertices[4]));
			check(grid2D.elements[0]->vertices[3]==&(grid2D.vertices[3]));
			// 1
			check(grid2D.elements[1]->vertices[0]==&(grid2D.vertices[1]));
			check(grid2D.elements[1]->vertices[1]==&(grid2D.vertices[2]));
			check(grid2D.elements[1]->vertices[2]==&(grid2D.vertices[5]));
			check(grid2D.elements[1]->vertices[3]==&(grid2D.vertices[4]));
		}
	}
}