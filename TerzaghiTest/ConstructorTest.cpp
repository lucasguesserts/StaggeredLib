#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>

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
		terzaghi.fluidViscosity = 1E-5;
		check(terzaghi.fluidViscosity==1E-5);
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
		terzaghi.insertPressureAccumulationTermInMatrix();
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
		terzaghi.insertPressureAccumulationTermInIndependent();
		for(auto element: terzaghi.grid.elements)
		{
			const unsigned index = terzaghi.getPindex(element);
			check(terzaghi.linearSystem.independent[index]==(diagonalValue*oldPressureValue));
		}
	}
}

TestCase("Pressure diffusive term", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	const unsigned linearSystemSize = terzaghi.numberOfElements + 3 * terzaghi.numberOfStaggeredElements;
	terzaghi.permeability = 20.0;
	terzaghi.fluidViscosity = 4.0;
	terzaghi.timeImplicitCoefficient = 0.7;
	terzaghi.timeInterval = 1.1;
	section("matrix")
	{
		Eigen::SparseMatrix<double> matrix(linearSystemSize,linearSystemSize);
		std::vector< Eigen::Triplet<double,unsigned> > triplets = {
			{ 0, 0,  11.55 },
			{ 0, 1, -11.55 },
			{ 1, 0, -11.55 },
			{ 1, 1,  11.55 }
		};
		matrix.setFromTriplets(triplets.begin(), triplets.end());
		terzaghi.insertPressureDiffusiveTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		std::vector<double> oldPressureValues = {13.0, 17.0};
		terzaghi.setOldPressure(oldPressureValues);
		terzaghi.insertPressureDiffusiveTermInIndependent();
		std::vector<double> independentValues = {19.8, -19.8};
		for(unsigned count=0 ; count<terzaghi.numberOfElements ; ++count)
		{
			auto element = terzaghi.grid.elements[count];
			auto independentIndex = terzaghi.getPindex(element);
			check(terzaghi.linearSystem.independent[independentIndex]==Approx(independentValues[count]));
		}
	}
}