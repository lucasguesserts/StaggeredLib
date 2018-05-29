#include <Utils/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

TestCase("Terzaghi constructor test", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "GridReaderTest_CGNS.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.viscosity = 1E-5;
	check(terzaghi.viscosity==1E-5);
}