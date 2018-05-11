#define CATCH_CONFIG_MAIN
#include <Utils/Test.hpp>

#include <cstdlib>
#include <ctime>

TestCase("Eigen initialize random generator")
{
	// it is here just to initialize the std random generator for tests
	std::srand(static_cast<unsigned int>(std::time(0)));
}