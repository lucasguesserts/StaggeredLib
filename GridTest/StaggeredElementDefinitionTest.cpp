#include <Utils/Test.hpp>

#include <Grid/StaggeredElementDefinition.hpp>

TestCase("Staggered element definition", "[StaggeredElementDefinition]")
{
	constexpr unsigned firstVertexIndex = 5;
	constexpr unsigned secondVertexIndex = 18;
	constexpr unsigned elementIndex = 91;
	StaggeredElementDefinition staggeredElementDefinition(firstVertexIndex,secondVertexIndex,elementIndex);
	section("constructor")
	{
		check(staggeredElementDefinition.vertices[0]==firstVertexIndex);
		check(staggeredElementDefinition.vertices[1]==secondVertexIndex);
		check(staggeredElementDefinition.elements[0]==elementIndex);
		check(staggeredElementDefinition.type==StaggeredElementDefinition::Type::Triangle);
	}
	section("add element")
	{
		constexpr unsigned secondElementIndex = 47;
		staggeredElementDefinition.addElement(secondElementIndex);
		check(staggeredElementDefinition.elements[1]==secondElementIndex);
		check(staggeredElementDefinition.type==StaggeredElementDefinition::Type::Quadrangle);
	}
	section("operator==")
	{
		check(staggeredElementDefinition==StaggeredElementDefinition(firstVertexIndex,secondVertexIndex,elementIndex));
		check(staggeredElementDefinition==StaggeredElementDefinition(firstVertexIndex,secondVertexIndex,elementIndex+1));
		check(staggeredElementDefinition==StaggeredElementDefinition(secondVertexIndex,firstVertexIndex,elementIndex+1));
		checkFalse(staggeredElementDefinition==StaggeredElementDefinition(firstVertexIndex+1,secondVertexIndex,elementIndex));
		checkFalse(staggeredElementDefinition==StaggeredElementDefinition(firstVertexIndex,secondVertexIndex+5,elementIndex));
	}
}