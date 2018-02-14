#include <Utils/Test.hpp>
#include <vector>
#include <string>
#include <iostream>

#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Element2D.hpp>
#include <GeometricEntity/Line.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/Quadrangle.hpp>
#include <Grid/GridData.hpp>
#include <Grid/GridReader.hpp>
#include <Grid/Grid.hpp>
#include <Grid/Grid2D.hpp>
#include <Grid/GridBuilder.hpp>

TestCase("Entity", "[Entity]")
{
	const unsigned handle = 3;
	section("constructor")
	{
		Entity entity(handle);
		check(entity.getHandle()==handle);
	}
	section("set handle")
	{
		Entity entity;
		entity.setHandle(handle);
		check(entity.getHandle()==handle);
	}
}

TestCase("Vertex constructor", "[Vertex]")
{
	const std::vector<double> values = {3.2, -5.7, 9.4};
	const unsigned handle = 4;
	Vertex vertex(values[0], values[1], values[2], handle);
	section("Handle")
	{
		check(vertex.getHandle()==handle);
		const unsigned newHandle = 7;
		vertex.setHandle(newHandle);
		check(vertex.getHandle()==newHandle);
	}
	section("constructor values")
	{
		for(unsigned i=0 ; i<3 ; ++i)
			check(vertex(i)==values[i]);
	}
	section("Eigen::Vector functionalities")
	{
		Eigen::Vector3d eigenVector(4.2, 9.3, -6.1);
		Eigen::Vector3d sumAnswer(7.4, 3.6, 3.3);
		Eigen::Vector3d sum = vertex + eigenVector;
		for(unsigned i=0 ; i<3 ; ++i)
		check(sum[i]==Approx(sumAnswer[i]));
	}
}

TestCase("Element2D compute triangle area vector", "[Element2D]")
{
	std::vector<Vertex> vertices;
	vertices.push_back(Vertex(-5,  4,  7, 0));
	vertices.push_back(Vertex( 9, -2,  4, 1));
	vertices.push_back(Vertex( 3,  5, -6, 2));
	const Eigen::Vector3d areaVector(40.5, 79, 31);
	Triangle triangle;
	for(Vertex& vertex: vertices)
		triangle.addVertex(vertex);
	check(triangle.getAreaVector()==areaVector);
}

TestCase("Line", "[Element][Element2D][Line]")
{
	const unsigned numberOfVertices = 2;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex( 2.0, -5.0,  3.0, 0));
	 	vertices.push_back(Vertex( 4.0, -1.0,  6.0, 1));
	const Eigen::Vector3d localCoordinates(0.4, 0.0, 0.0);
	Line line;
	for(Vertex& vertex: vertices)
		line.addVertex(vertex);
	section("Basic requirements")
	{
		const unsigned dimension = 1;
		require(line.getNumberOfVertices()==numberOfVertices);
		require(line.dimension==dimension);
	}
	section("Centroid")
	{
		const Eigen::Vector3d centroid(3, -3, 4.5);
		check(line.getCentroid()==centroid);
	}
	section("Volume")
	{
		const double volume = 5.385164807134504; // the length
		check(line.getVolume()==Approx(volume));
	}
	section("Shape function values")
	{
		Eigen::Vector2d shapeFunctionValues(0.6, 0.4);
		Eigen::Vector2d lineShapeFunctionValues = line.getShapeFunctionValues(localCoordinates);
		for(unsigned i=0 ; i<2 ; ++i)
			check(lineShapeFunctionValues(i)==Approx(shapeFunctionValues(i)));
	}
	section("Shape function derivatives")
	{
		const unsigned numberOfRows = 2;
		const unsigned numberOfColumns = 3;
		Eigen::MatrixXd shapeFunctionDerivatives(numberOfRows, numberOfColumns);
			shapeFunctionDerivatives << -1.0, 0.0, 0.0,
		                                 1.0, 0.0, 0.0;
		Eigen::MatrixXd lineShapeFunctionDerivatives = line.getShapeFunctionDerivatives(localCoordinates);
		for(unsigned i=0 ; i<numberOfRows ; ++i)
			for(unsigned j=0 ; j<numberOfColumns ; ++j)
				check(lineShapeFunctionDerivatives(i,j)==Approx(shapeFunctionDerivatives(i,j)));
	}
}

TestCase("Triangle", "[Element][Element2D][Triangle]")
{
	const unsigned numberOfVertices = 3;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex(2.0, -5.0, 3.0, 0));
		vertices.push_back(Vertex(3.0, 7.0, -2.0, 1));
		vertices.push_back(Vertex(-4.0, -1.0, 6.0, 2));
	const Eigen::Vector3d localCoordinates(0.2, 0.3, 0.0);
	Triangle triangle;
	for(Vertex& vertex: vertices)
		triangle.addVertex(vertex);
	section("Basic requirements")
	{
		require(triangle.getNumberOfVertices()==3);
		require(triangle.dimension==2);
	}
	section("Centroid")
	{
		const Eigen::Vector3d centroid(1.0/3.0, 1.0/3.0, 7.0/3.0);
		check(triangle.getCentroid()==centroid);
	}
	section("Area vector")
	{
		const Eigen::Vector3d areaVector(28.0, 13.5, 38.0);
		check(triangle.getAreaVector()==areaVector);
	}
	section("Volume")
	{
		const double volume = 49.09429702; // the area
		check(triangle.getVolume()==Approx(volume));
	}
	section("Shape function values")
	{
		Eigen::VectorXd shapeFunctionValues(3); shapeFunctionValues << 0.5, 0.2, 0.3;
		check(triangle.getShapeFunctionValues(localCoordinates)==shapeFunctionValues);
	}
	section("Shape function derivatives")
	{
		Eigen::MatrixXd shapeFunctionDerivatives(3,3);
			shapeFunctionDerivatives << -1.0, -1.0, 0.0,
										 1.0,  0.0, 0.0,
										 0.0,  1.0, 0.0;
		check(triangle.getShapeFunctionDerivatives(localCoordinates)==shapeFunctionDerivatives);
	}
}

TestCase("Quadrangle", "[Element][Element2D][Quadrangle]")
{
	const unsigned numberOfVertices = 4;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex( 2.0, -5.0,  3.0, 0));
		vertices.push_back(Vertex( 4.0, -1.0,  6.0, 1));
		vertices.push_back(Vertex( 4.0,  3.0,  3.4, 2));
		vertices.push_back(Vertex( 3.0,  7.0, -2.0, 3));
	const Eigen::Vector3d localCoordinates(0.2, 0.3, 0.0);
	Quadrangle quadrangle;
	for(Vertex& vertex: vertices)
		quadrangle.addVertex(vertex);
	section("Basic requirements")
	{
		const unsigned dimension = 2;
		require(quadrangle.getNumberOfVertices()==numberOfVertices);
		require(quadrangle.dimension==dimension);
	}
	section("Centroid")
	{
		const Eigen::Vector3d centroid(3.25, 1.0, 2.6);
		check(quadrangle.getCentroid()==centroid);
	}
	section("Area vector")
	{
		const Eigen::Vector3d areaVector(-33.6, 7.8, 12);
		Eigen::Vector3d quadrangleAreaVector = quadrangle.getAreaVector();
		for(unsigned i=0 ; i<3 ; ++i)
			check(quadrangleAreaVector(i)==Approx(areaVector(i)));
	}
	section("Volume")
	{
		const double volume = 36.5212267044797; // the area
		check(quadrangle.getVolume()==Approx(volume));
	}
	section("Shape function values")
	{
		Eigen::Vector4d shapeFunctionValues(0.56, 0.14, 0.06, 0.24);
		Eigen::Vector4d quadrangleShapeFunctionValues = quadrangle.getShapeFunctionValues(localCoordinates);
		for(unsigned i=0 ; i<4 ; ++i)
			check(quadrangleShapeFunctionValues(i)==Approx(shapeFunctionValues(i)));
	}
	section("Shape function derivatives")
	{
		const unsigned numberOfRows = 4;
		const unsigned numberOfColumns = 3;
		Eigen::MatrixXd shapeFunctionDerivatives(numberOfRows, numberOfColumns);
			shapeFunctionDerivatives << -0.7, -0.8, 0.0,
		                                 0.7, -0.2, 0.0,
		                                 0.3,  0.2, 0.0,
		                                -0.3,  0.8, 0.0;
		Eigen::MatrixXd quadrangleShapeFunctionDerivatives = quadrangle.getShapeFunctionDerivatives(localCoordinates);
		for(unsigned i=0 ; i<numberOfRows ; ++i)
			for(unsigned j=0 ; j<numberOfColumns ; ++j)
				check(quadrangleShapeFunctionDerivatives(i,j)==Approx(shapeFunctionDerivatives(i,j)));
	}
}

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

TestCase("grid reader from CGNS", "[Grid][GridReader][CGNS]")
{
	const std::string cgnsGridFileName = GridReader::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	GridData gridData = GridReader::CGNS(cgnsGridFileName);
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
	Grid grid;
	grid.dimension = dimension;
	grid.vertices.reserve(numberOfVertices);
		grid.vertices.push_back(Vertex( 2.0, -5.0,  3.0, 0));
		grid.vertices.push_back(Vertex( 4.0, -1.0,  6.0, 1));
		grid.vertices.push_back(Vertex( 4.0,  3.0,  3.4, 2));
		grid.vertices.push_back(Vertex( 3.0,  7.0, -2.0, 3));
	const unsigned numberOfElements = 1;
	Quadrangle quadrangle;
	quadrangle.addVertex(grid.vertices[0]);
	quadrangle.addVertex(grid.vertices[1]);
	quadrangle.addVertex(grid.vertices[2]);
	quadrangle.addVertex(grid.vertices[3]);
	grid.elements.push_back(&quadrangle);
	check(grid.vertices.size()==numberOfVertices);
	check(grid.elements.size()==numberOfElements);
	check(grid.dimension==dimension);
}

TestCase("Grid builder 2D", "[GridBuilder][Grid][Grid2D]")
{
	const std::string cgnsGridFileName = GridReader::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	GridData gridData = GridReader::CGNS(cgnsGridFileName);
	Grid2D grid2D = GridBuilder::build2D(gridData);
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
			// It is necessary to test the handles.
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
			// It is necessary to test the handles.
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
