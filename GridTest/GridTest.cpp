#include <Utils/Test.hpp>

#include <Grid/Entity.hpp>

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
