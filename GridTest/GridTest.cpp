#include <Utils/Test.hpp>
#include <vector>

#include <Grid/Entity.hpp>
#include <Grid/Vertex.hpp>
#include <Grid/VertexCollection.hpp>

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
