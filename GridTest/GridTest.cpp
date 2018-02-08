#include <Utils/Test.hpp>
#include <vector>

#include <Grid/Entity.hpp>
#include <Grid/Vertex.hpp>
#include <Grid/VertexCollection.hpp>
#include <Grid/Element.hpp>
#include <Grid/Element2D.hpp>
#include <Grid/Line.hpp>
#include <Grid/Triangle.hpp>
#include <Grid/Quadrangle.hpp>
#include <Grid/GridData.hpp>

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

TestCase("Vertex default constructor", "[Vertex]")
{
	const std::vector<double> defaultValues = {0.0, 0.0, 0.0};
	const unsigned defaultHandle = 0;
	Vertex vertex;
	for(unsigned i=0 ; i<3 ; ++i)
		check(vertex(i)==defaultValues[i]);
	check(vertex.getHandle()==defaultHandle);
}

TestCase("Vertex", "[Vertex]")
{
	const unsigned handle = 4;
	Vertex vertex;
	vertex.setHandle(handle);
	check(vertex.getHandle()==handle);
}

TestCase("Vertex explicit constructor", "[Vertex]")
{
	const std::vector<double> values = {3.2, 5.7, 9.4};
	const unsigned handle = 4;
	Vertex vertex(values[0], values[1], values[2], handle);
	for(unsigned i=0 ; i<3 ; ++i)
		check(vertex(i)==values[i]);
	check(vertex.getHandle()==handle);
}

TestCase("Vertex collection", "[VertexCollection]")
{
	// Create vector
	const unsigned numberOfVertices = 5;
	std::vector<Vertex> vertexVector(numberOfVertices);
	// Create collection
	VertexCollection vertexCollection(numberOfVertices);
	for(Vertex& vertex: vertexVector)
		vertexCollection.addVertex(vertex);
	// Set vector vertex values
	for(unsigned i=0 ; i<numberOfVertices ; ++i)
	{
		vertexVector[i] << i, i, i; // Eigen way of setting vector values
		vertexVector[i].setHandle(i);
	}
	section("Vertex values")
	{
		for(unsigned i = 0; i<numberOfVertices ; ++i)
		{
			Vertex vertex = vertexCollection.getVertex(i);
			check(vertex.x()==i);
			check(vertex.y()==i);
			check(vertex.z()==i);
			check(vertex.getHandle()==i);
		}
	}
	section("Vertex pointers")
	{
		std::vector<const Vertex *> vertexCollectionVector = vertexCollection.getVertices();
		for(unsigned i=0 ; i<numberOfVertices ; ++i)
			check(&vertexVector[i]==vertexCollectionVector[i]);
	}
}

TestCase("Element2D compute triangle area vector", "[Element2D]")
{
	std::vector<Vertex> vertices(3);
	vertices[0] = Vertex(-5,  4,  7, 0);
	vertices[1] = Vertex( 9, -2,  4, 1);
	vertices[2] = Vertex( 3,  5, -6, 2);
	const Eigen::Vector3d areaVector(40.5, 79, 31);
	Triangle triangle;
	for(Vertex& vertex: vertices)
		triangle.addVertex(vertex);
	check(triangle.getAreaVector()==areaVector);
}

TestCase("Line", "[Element][Element2D][Line]")
{
	const unsigned numberOfVertices = 2;
	std::vector<Vertex> vertices(numberOfVertices);
		vertices[0] = Vertex( 2.0, -5.0,  3.0, 0);
		vertices[1] = Vertex( 4.0, -1.0,  6.0, 1);
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
	std::vector<Vertex> vertices(numberOfVertices);
		vertices[0] = Vertex(2.0, -5.0, 3.0, 0);
		vertices[1] = Vertex(3.0, 7.0, -2.0, 1);
		vertices[2] = Vertex(-4.0, -1.0, 6.0, 2);
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
	std::vector<Vertex> vertices(numberOfVertices);
		vertices[0] = Vertex( 2.0, -5.0,  3.0, 0);
		vertices[1] = Vertex( 4.0, -1.0,  6.0, 1);
		vertices[2] = Vertex( 4.0,  3.0,  3.4, 2);
		vertices[3] = Vertex( 3.0,  7.0, -2.0, 3);
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
	const unsigned numberOfVertices = 5;
	GridData gridData;
	gridData.coordinates.resize(numberOfVertices,Eigen::NoChange);
	gridData.coordinates <<
		0,   0, 0,
		1,   0, 0,
		0,   1, 0,
		1,   1, 0,
		0.5, 2, 0;
	section("dimension")
	{
		const unsigned dimension = 2;
		gridData.dimension = dimension;
		check(gridData.dimension==dimension);
	}
	section("quadrangle connectivity")
	{
		Eigen::Matrix<unsigned,1,4> quadrangleConnectivity;
		quadrangleConnectivity << 0, 1, 3, 2;
		gridData.quadrangleConnectivity.resize(1,Eigen::NoChange);
		gridData.quadrangleConnectivity << 0, 1, 3, 2;
		check(quadrangleConnectivity==quadrangleConnectivity);
	}
	section("Triangle connectivity")
	{
		Eigen::Matrix<unsigned,1,3> triangleConnectivity;
		triangleConnectivity << 2, 3, 4;
		gridData.triangleConnectivity.resize(1,Eigen::NoChange);
		gridData.triangleConnectivity << 2, 3, 4;
		check(triangleConnectivity==triangleConnectivity);
	}
}
