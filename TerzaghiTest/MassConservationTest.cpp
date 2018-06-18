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
		const std::vector<double> oldPressureValues = {2.0, 3.0};
		terzaghi.setOldPressure(oldPressureValues);
		terzaghi.insertPressureAccumulationTermInIndependent();
		for(unsigned count=0 ; count< terzaghi.grid.elements.size() ; ++count)
		{
			auto element = terzaghi.grid.elements[count];
			const unsigned index = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.independent[index]==(diagonalValue*oldPressureValues[count]));
		}
	}
}

TestCase("Pressure diffusive term", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	const unsigned linearSystemSize = terzaghi.numberOfElements + 2 * terzaghi.numberOfStaggeredElements;
	terzaghi.permeability = 20.0;
	terzaghi.fluidViscosity = 4.0;
	terzaghi.timeImplicitCoefficient = 0.7;
	terzaghi.timeInterval = 1.1;
	terzaghi.pressureGradient[2] = { {0.0, {-3.0, 2.0, 0.0}}, {1.0, {4.0, -5.0, 0.0}} };
	section("matrix")
	{
		Eigen::SparseMatrix<double> matrix(linearSystemSize,linearSystemSize);
		std::vector< Eigen::Triplet<double,unsigned> > triplets = {
			{ 0, 0, -38.5 },
			{ 0, 1, +69.3 },
			{ 1, 0, +38.5 },
			{ 1, 1, -69.3 }
		};
		matrix.setFromTriplets(triplets.begin(), triplets.end());
		terzaghi.insertPressureDiffusiveTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		std::vector<double> oldPressureValues = {2.0, 3.0};
		terzaghi.setOldPressure(oldPressureValues);
		terzaghi.insertPressureDiffusiveTermInIndependent();
		std::vector<double> independentValues = {-56.1, +56.1};
		for(unsigned count=0 ; count<terzaghi.numberOfElements ; ++count)
		{
			auto element = terzaghi.grid.elements[count];
			auto independentIndex = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.independent[independentIndex]==Approx(independentValues[count]));
		}
	}
}

TestCase("Pressure volumetric dilatation term", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.alpha = 2.0;
	section("matrix")
	{
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			{0,  2,  0.0},
			{0,  7, -4.0},
			{0,  3, +4.0},
			{0,  8,  0.0},
			{0,  4, -4.0},
			{0,  9, +4.0},
			{1,  4, +4.0},
			{1,  9, -4.0},
			{1,  5,  0.0},
			{1, 10, +4.0},
			{1,  6, -4.0},
			{1, 11,  0.0}
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
			{ 1.0,  3.0, 0.0},
			{ 7.0, 11.0, 0.0},
			{17.0, 19.0, 0.0},
			{29.0, 31.0, 0.0},
			{41.0, 43.0, 0.0},
		};
		terzaghi.setOldDisplacement(displacements);
		terzaghi.insertPressureVolumeDilatationTermInIndependent();
		std::vector<double> independentValues = {24.0, -48.0};
		for(unsigned count=0 ; count<terzaghi.numberOfElements ; ++count)
		{
			auto element = terzaghi.grid.elements[count];
			auto independentIndex = terzaghi.transformIndex(Component::P,element);
			check(terzaghi.linearSystem.independent[independentIndex]==Approx(independentValues[count]));
		}
	}
}

TestCase("Mass conservation - pressure dirichlet boundary condition", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.permeability = 20.0;
	terzaghi.fluidViscosity = 4.0;
	terzaghi.timeImplicitCoefficient = 0.7;
	terzaghi.timeInterval = 1.1;
	terzaghi.pressureGradient = {
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, {-3.0, +5.0, 0.0}}, {1, {7.0, +11.0, 0.0}} },
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, {-4.0, +3.0, 0.0}}, {1, {5.0,  -2.0, 0.0}} },
		{ {0, Eigen::Vector3d::Zero()} }
	};
	terzaghi.pressureGradientIndependent = {
		Eigen::Vector3d::Zero(),
		{-13.0, -17.0, 0.0},
		Eigen::Vector3d::Zero(),
		{-5.0, 7.0, 0.0},
		Eigen::Vector3d::Zero()
	};
	terzaghi.setOldPressure(std::vector<double>{11.0, 17.0});
	terzaghi.boundaries[3].isPressureDirichlet = true;
	terzaghi.boundaries[3].pressurePrescribedValue = 7.0;
	terzaghi.boundaries[0].isPressureDirichlet = true;
	terzaghi.boundaries[0].pressurePrescribedValue = 5.0;
	section("matrix")
	{
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			{0, 0, +23.1},
			{0, 1, -53.9},
			{1, 0, -23.1},
			{1, 1, +15.4}
		};
		Eigen::SparseMatrix<double> matrix(terzaghi.linearSystemSize,terzaghi.linearSystemSize);
		matrix.setFromTriplets(triplets.cbegin(), triplets.cend());
		terzaghi.insertPressureDirichletBoundaryConditionToMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
	section("prescribed value in independent")
	{
		terzaghi.setOldPressure([](Eigen::Vector3d) -> double { return 0.0;});
		terzaghi.insertPressureDirichletBoundaryConditionToIndependent();
		check(terzaghi.linearSystem.independent[0]==Approx(-1001.0));
		check(terzaghi.linearSystem.independent[1]==Approx(+385.0));
	}
	section("old value in independent")
	{
		terzaghi.boundaries[0].pressurePrescribedValue = 0.0;
		terzaghi.boundaries[3].pressurePrescribedValue = 0.0;
		terzaghi.insertPressureDirichletBoundaryConditionToIndependent();
		check(terzaghi.linearSystem.independent[0]==Approx(+283.8));
		check(terzaghi.linearSystem.independent[1]==Approx(-3.3));
	}
	section("prescribed and old pressure values in independent")
	{
		terzaghi.insertPressureDirichletBoundaryConditionToIndependent();
		check(terzaghi.linearSystem.independent[0]==Approx(-717.2));
		check(terzaghi.linearSystem.independent[1]==Approx(+381.7));
	}
}

TestCase("Mass conservation - pressure Neumann boundary condition", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.permeability = 3.0;
	terzaghi.fluidViscosity = 5.0;
	terzaghi.timeInterval = 1.1;

	terzaghi.boundaries[0].isPressureDirichlet = false;
	terzaghi.boundaries[0].pressureGradient << +4.7, -2.4, 0.0;
	terzaghi.boundaries[1].isPressureDirichlet = false;
	terzaghi.boundaries[1].pressureGradient << -6.0, -8.3, 0.0;
	terzaghi.boundaries[2].isPressureDirichlet = false;
	terzaghi.boundaries[2].pressureGradient << +7.0, -11.0, 0.0;
	terzaghi.boundaries[3].isPressureDirichlet = false;
	terzaghi.boundaries[3].pressureGradient << -2.7, +7.1, 0.0;

	terzaghi.insertPressureNeumannBoundaryConditionToIndependent();

	check(terzaghi.linearSystem.independent[0]==Approx(+7.392));
	check(terzaghi.linearSystem.independent[1]==Approx(-12.408));
}