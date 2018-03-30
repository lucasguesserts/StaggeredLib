#include <Utils/Test.hpp>
#include <array>

#include <GeometricEntity/Vertex.hpp>

TestCase("Vertex constructor", "[Vertex]")
{
	const std::array<double,3> values = {3.2, -5.7, 9.4};
	const unsigned index = 4;
	Vertex vertex(values[0], values[1], values[2], index);
	section("Index")
	{
		check(vertex.getIndex()==index);
		const unsigned newIndex = 7;
		vertex.setIndex(newIndex);
		check(vertex.getIndex()==newIndex);
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

TestCase("Vertex operator==")
{
	std::array<Vertex,3> vertices = {{
		{2.3, 5.7, 1.9999999, 0},
		{2.3, 5.7, 2.0000000, 0},
		{2.3, 5.7, -1.000000, 0},
	}};
	check(vertices[0]==vertices[1]);
	checkFalse(vertices[0]==vertices[2]);
}