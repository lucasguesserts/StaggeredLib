#include <Utils/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

TestCase("Terzaghi constructor test", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "GridReaderTest_CGNS.cgns";
	Terzaghi terzaghi(gridFile);
	section("sizes")
	{
		constexpr unsigned linearSystemSize = 48;
		check(terzaghi.linearSystem.matrix.rows()==linearSystemSize);
		check(terzaghi.linearSystem.matrix.cols()==linearSystemSize);
		check(terzaghi.linearSystem.independent.size()==linearSystemSize);
	}
	section("phisical quantities")
	{
		terzaghi.viscosity = 1E-5;
		check(terzaghi.viscosity==1E-5);
	}
}