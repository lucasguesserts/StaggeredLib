#include <Utils/Test.hpp>
#include <vector>

#include <Grid/Entity.hpp>
#include <Grid/Vertex.hpp>

TestCase("Entity", "[entity]")
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

TestCase("Vertex default constructor", "[vertex]")
{
	const std::vector<double> defaultValues = {0.0, 0.0, 0.0};
	const unsigned defaultHandle = 0;
	Vertex vertex;
	for(unsigned i=0 ; i<3 ; ++i)
		check(vertex(i)==defaultValues[i]);
	check(vertex.getHandle()==defaultHandle);
}

TestCase("Vertex", "[vertex]")
{
	const unsigned handle = 4;
	Vertex vertex;
	vertex.setHandle(handle);
	check(vertex.getHandle()==handle);
}

TestCase("Vertex explicit constructor", "[vertex]")
{
	const std::vector<double> values = {3.2, 5.7, 9.4};
	const unsigned handle = 4;
	Vertex vertex(values[0], values[1], values[2], handle);
	for(unsigned i=0 ; i<3 ; ++i)
		check(vertex(i)==values[i]);
	check(vertex.getHandle()==handle);
}
