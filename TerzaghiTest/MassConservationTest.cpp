#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

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
			const unsigned index = terzaghi.transformIndex(Component::P,element);
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
			const unsigned index = terzaghi.transformIndex(Component::P,element);
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
			auto independentIndex = terzaghi.transformIndex(Component::P,element);
			// check(terzaghi.linearSystem.independent[independentIndex]==Approx(independentValues[count]));
			// TODO: compute the staggered triangle gradients
		}
	}
}

TestCase("Pressure volumetric dilatation term", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.alpha = 1.0;
	section("matrix")
	{
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			{0,  2,  0.0},
			{0,  7, -2.0},
			{0, 12,  0.0},
			{0,  3,  2.0},
			{0,  8,  0.0},
			{0, 13,  0.0},
			{0,  4, -2.0},
			{0,  9,  2.0},
			{0, 14,  0.0},
			{1,  4,  2.0},
			{1,  9, -2.0},
			{1, 14,  0.0},
			{1,  5,  0.0},
			{1, 10,  2.0},
			{1, 15,  0.0},
			{1,  6, -2.0},
			{1,  1,  0.0},
			{1, 16,  0.0},
		};
		Eigen::SparseMatrix<double> matrix(terzaghi.linearSystemSize,terzaghi.linearSystemSize);
		matrix.setFromTriplets(triplets.cbegin(), triplets.cend());
		terzaghi.insertPressureVolumeDilatationTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		std::vector<Eigen::Vector3d> displacements = {
			{ 1.0,  3.0,  5.0},
			{ 7.0, 11.0, 13.0},
			{17.0, 19.0, 23.0},
			{29.0, 31.0, 37.0},
			{41.0, 43.0, 47.0},
		};
		terzaghi.setOldDisplacement(displacements);
		terzaghi.insertPressureVolumeDilatationTermInIndependent();
		std::vector<double> independentValues = {12.0, -24.0};
		for(unsigned count=0 ; count<terzaghi.numberOfElements ; ++count)
		{
			auto element = terzaghi.grid.elements[count];
			auto independentIndex = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.independent[independentIndex]==Approx(independentValues[count]));
		}
	}
}