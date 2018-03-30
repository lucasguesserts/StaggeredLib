#include <Utils/Test.hpp>
#include <Utils/contains.hpp>

TestCase("contains")
{
	section("int")
	{
		std::vector<int> vec{1, 2, 3, 4, 5};
		check(contains(vec,3));
	}
	section("double")
	{
		std::vector<double> vec{2.3, 7.3, -1.9, 6.4};
		check(contains(vec,-1.9));
	}
	section("char")
	{
		std::vector<char> vec{'a', 'c', '\0', '5'};
		check(contains(vec,'\0'));
		check(contains(vec,'5'));
	}
}