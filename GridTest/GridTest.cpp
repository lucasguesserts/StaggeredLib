#include <Utils/Test.hpp>
#include <vector>

#include <Grid/Entity.hpp>
#include <Grid/Vertex.hpp>
#include <Grid/VertexCollection.hpp>
#include <Grid/Element.hpp>
#include <Grid/Element2D.hpp>
#include <Grid/Triangle.hpp>

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
		std::vector<Vertex *> vertexCollectionVector = vertexCollection.getVertices();
		for(unsigned i=0 ; i<numberOfVertices ; ++i)
			check(&vertexVector[i]==vertexCollectionVector[i]);
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
