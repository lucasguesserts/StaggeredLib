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

TestCase("Pressure accumulation term")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.porosity = 0.1;
	terzaghi.alpha = 0.9;
	terzaghi.fluidCompressibility = 3;
	terzaghi.solidCompressibility = 4;
	constexpr double volume = 2;
	constexpr double diagonalValue = 7.0;
	section("matrix")
	{
		terzaghi.insertPressureAccumulationTermToMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		for(auto element: terzaghi.grid.elements)
		{
			const unsigned index = terzaghi.getPindex(element);
			check(terzaghi.linearSystem.matrix.coeff(index,index)==diagonalValue);
		}
	}
	section("independent")
	{
		constexpr double oldPressureValue = 2.0;
		terzaghi.setOldPressure([oldPressureValue](Eigen::Vector3d) -> double {return oldPressureValue;});
		terzaghi.insertPressureAccumulationTermToIndependent();
		for(auto element: terzaghi.grid.elements)
		{
			const unsigned index = terzaghi.getPindex(element);
			check(terzaghi.linearSystem.independent[index]==(diagonalValue*oldPressureValue));
		}
	}
}