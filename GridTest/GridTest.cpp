#include <Utils/Test.hpp>
#include <vector>
#include <string>

#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid.hpp>
#include <Grid/Grid2D.hpp>

TestCase("Grid data structure", "[Grid][GridData]")
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
		Eigen::Matrix<unsigned,2,4> quadrangleConnectivity;
		quadrangleConnectivity << 0, 1, 3, 2,
		                          3, 5, 6, 4;
		gridData.quadrangleConnectivity.resize(2,Eigen::NoChange);
		gridData.quadrangleConnectivity << 0, 1, 3, 2,
		                                   3, 5, 6, 4;
		check(quadrangleConnectivity==quadrangleConnectivity);
	}
	section("Triangle connectivity")
	{
		Eigen::Matrix<unsigned,2,3> triangleConnectivity;
		triangleConnectivity << 2, 3, 4,
		                        1, 5, 3;
		gridData.triangleConnectivity.resize(2,Eigen::NoChange);
		gridData.triangleConnectivity << 2, 3, 4,
		                                 1, 5, 3;
		check(triangleConnectivity==triangleConnectivity);
	}
}

TestCase("grid reader from CGNS", "[GridData][Grid][CGNS]")
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
		const unsigned numberOfQuadrangles = 2;
		const unsigned verticesPerQuadrangle = 4;
		Eigen::Matrix<unsigned,Eigen::Dynamic,Eigen::Dynamic> quadrangleConnectivity;
		quadrangleConnectivity.resize(numberOfQuadrangles,verticesPerQuadrangle);
		quadrangleConnectivity << 0, 1, 4, 3,
		                          1, 2, 5, 4;
		check(gridData.quadrangleConnectivity==quadrangleConnectivity);
	}
	section("triangle connectivity")
	{
		const unsigned numberOfTriangles = 4;
		const unsigned verticesPerTriangle = 3;
		Eigen::Matrix<unsigned,Eigen::Dynamic,Eigen::Dynamic> triangleConnectivity;
		triangleConnectivity.resize(numberOfTriangles,verticesPerTriangle);
		triangleConnectivity << 3, 4, 7,
		                        3, 7, 6,
		                        4, 5, 8,
		                        4, 8, 7;
		check(gridData.triangleConnectivity==triangleConnectivity);
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
		const unsigned numberOfTriangles = 4;
		section("triangular elements centroid")
		{
			Eigen::Vector3d triangleCentroid[4];
			triangleCentroid[0] << 1.0, 2.0, 0.0;
			triangleCentroid[1] << 0.5, 2.5, 0.0;
			triangleCentroid[2] << 2.5, 2.0, 0.0;
			triangleCentroid[3] << 2.0, 2.5, 0.0;
			for(unsigned elementIndex=0 ; elementIndex<numberOfTriangles ; ++elementIndex)
				check(grid2D.elements[elementIndex]->getCentroid()==triangleCentroid[elementIndex]);
		}
		section("elements vertices")
		{
			// 0
			check(grid2D.elements[0]->vertices[0]==&(grid2D.vertices[3]));
			check(grid2D.elements[0]->vertices[1]==&(grid2D.vertices[4]));
			check(grid2D.elements[0]->vertices[2]==&(grid2D.vertices[7]));
			// 1
			check(grid2D.elements[1]->vertices[0]==&(grid2D.vertices[3]));
			check(grid2D.elements[1]->vertices[1]==&(grid2D.vertices[7]));
			check(grid2D.elements[1]->vertices[2]==&(grid2D.vertices[6]));
			// 2
			check(grid2D.elements[2]->vertices[0]==&(grid2D.vertices[4]));
			check(grid2D.elements[2]->vertices[1]==&(grid2D.vertices[5]));
			check(grid2D.elements[2]->vertices[2]==&(grid2D.vertices[8]));
			//3
			check(grid2D.elements[3]->vertices[0]==&(grid2D.vertices[4]));
			check(grid2D.elements[3]->vertices[1]==&(grid2D.vertices[8]));
			check(grid2D.elements[3]->vertices[2]==&(grid2D.vertices[7]));
		}
	}
	section("quadrangles")
	{
		const unsigned numberOfQuadrangles = 2;
			section("quadrangular elements centroid")
		{
			Eigen::Vector3d quadrangleCentroid[numberOfQuadrangles];
			quadrangleCentroid[0] << 0.75, 0.75, 0.0;
			quadrangleCentroid[1] << 2.25, 0.75, 0.0;
			check(grid2D.elements[4]->getCentroid()==quadrangleCentroid[0]);
			check(grid2D.elements[5]->getCentroid()==quadrangleCentroid[1]);
		}
		section("elements vertices")
		{
			// 4
			check(grid2D.elements[4]->vertices[0]==&(grid2D.vertices[0]));
			check(grid2D.elements[4]->vertices[1]==&(grid2D.vertices[1]));
			check(grid2D.elements[4]->vertices[2]==&(grid2D.vertices[4]));
			check(grid2D.elements[4]->vertices[3]==&(grid2D.vertices[3]));
			// 5
			check(grid2D.elements[5]->vertices[0]==&(grid2D.vertices[1]));
			check(grid2D.elements[5]->vertices[1]==&(grid2D.vertices[2]));
			check(grid2D.elements[5]->vertices[2]==&(grid2D.vertices[5]));
			check(grid2D.elements[5]->vertices[3]==&(grid2D.vertices[4]));
		}
	}
}
