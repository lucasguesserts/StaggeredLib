#include <Utils/Test.hpp>
#include <Eigen/Core>

#include <CGNSFile/ElementDefinition.hpp>

TestCase("Element definition", "[ElementDefinition]")
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
