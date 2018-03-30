#include <Utils/Test.hpp>

#include <GeometricEntity/Entity.hpp>

TestCase("Entity", "[Entity]")
{
	const unsigned index = 3;
	section("constructor")
	{
		Entity entity(index);
		check(entity.getIndex()==index);
	}
	section("set index")
	{
		Entity entity;
		entity.setIndex(index);
		check(entity.getIndex()==index);
	}
}