#include <Utils/Test.hpp>
#include <array>
#include <vector>
#include <string>

#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <Grid/ElementDefinition.hpp>
#include <Grid/CGNSFile.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid.hpp>
#include <Grid/Grid2D.hpp>

TestCase("Element definition for GridData", "[ElementDefinition]")
{
	constexpr unsigned numberOfVertices = 4;
	constexpr unsigned index = 7;
	Eigen::Matrix<unsigned,numberOfVertices,1> connectivity;
	connectivity << 1, 7, 4, 2;
	ElementDefinition<numberOfVertices> elementDefinition;
	elementDefinition.index = index;
	elementDefinition.connectivity << 1, 7, 4, 2;
	check(elementDefinition.index==index);
	check(elementDefinition.connectivity.size()==numberOfVertices);
	check(elementDefinition.connectivity==connectivity);
}

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

TestCase("CGNS file structure", "[CGNSFile]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	section("basic cgns file opening")
	{
		check(cgnsFile.fileIndex==1);
		check(cgnsFile.baseIndex==1);
		check(cgnsFile.zoneIndex==1);
		check(cgnsFile.cellDimension==2);
		check(cgnsFile.physicalDimension==2);
		check(cgnsFile.numberOfVertices==9);
		check(cgnsFile.numberOfElements==6);
	}
}

TestCase("grid reader from CGNS", "[GridData][CGNS]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	GridData gridData(cgnsGridFileName);
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

TestCase("Grid structure", "[Grid]")
{
	const unsigned dimension = 2;
	const unsigned numberOfVertices = 4;
	GridData gridData;
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
		for(unsigned i=0 ; i<3 ; ++i)
			check(grid.vertices[vertexIndex](i)==gridData.coordinates(vertexIndex,i));
		check(grid.vertices[vertexIndex].getIndex()==vertexIndex);
	}
}

TestCase("Grid 2D build", "[Grid][Grid2D]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	GridData gridData(cgnsGridFileName);
	Grid2D grid2D(gridData);
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
		for(unsigned i=0 ; i<grid2D.vertices.size() ; ++i)
			check(grid2D.vertices[i]==vertices[i]);
	}
	section("Elements index")
	{
		for(unsigned elementIndex=0 ; elementIndex<grid2D.elements.size() ; ++elementIndex)
			check(grid2D.elements[elementIndex]->getIndex()==elementIndex);
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
