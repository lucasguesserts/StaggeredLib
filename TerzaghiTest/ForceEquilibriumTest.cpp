#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

TestCase("Displacement stress term")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.shearModulus = 3.6;
	terzaghi.poissonCoefficient = 0.2;
	section("physical properties matrix")
	{
		Eigen::MatrixXd physicalPropertiesMatrix(6,6);
		physicalPropertiesMatrix <<
			9.6, 2.4, 0.0, 0.0, 0.0, 0.0,
			2.4, 9.6, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 3.6, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		check(terzaghi.getPhysicalPropertiesMatrix()==physicalPropertiesMatrix);
	}
	section("matrix")
	{
		terzaghi.displacementGradient = {
			{ {0, {5.3, -1.4, 0.0}}, {2, {-8.8, -2.9, 0.0}} },
			{ {0, Eigen::Vector3d::Zero()} },
			{ {0, Eigen::Vector3d::Zero()} },
			{ {0, Eigen::Vector3d::Zero()} },
			{ {1, {-2.9, 5.6, 0.0}}, {2, {4.2, -5.5, 0.0}} },
			{ {0, Eigen::Vector3d::Zero()} }
		};
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			// face 0
				// staggered element 0
				{2, 2, -40.64},
				{2, 4, +42.40},
				{2, 7, +27.68},
				{2, 9, -37.60},
				{7, 2, +20.32},
				{7, 4, -21.20},
				{7, 7, -30.64},
				{7, 9, -16.00},
				// staggered element 2
				{4, 2, +40.64},
				{4, 4, -42.40},
				{4, 7, -27.68},
				{4, 9, +37.60},
				{9, 2, -20.32},
				{9, 4, +21.20},
				{9, 7, +30.64},
				{9, 9, +16.00},
			// face 4
				// staggered element 3
				{ 5, 3, -45.44},
				{ 5, 4, +53.28},
				{ 5, 8, +22.88},
				{ 5, 9, -28.96},
				{10, 3, +22.72},
				{10, 4, -26.64},
				{10, 8, -78.64},
				{10, 9, +80.48},
				// staggered element 2
				{ 4, 3, +45.44},
				{ 4, 4, -53.28},
				{ 4, 8, -22.88},
				{ 4, 9, +28.96},
				{ 9, 3, -22.72},
				{ 9, 4, +26.64},
				{ 9, 8, +78.64},
				{ 9, 9, -80.48},
		};
		Eigen::SparseMatrix<double> matrix(terzaghi.linearSystemSize,terzaghi.linearSystemSize);
		matrix.setFromTriplets(triplets.cbegin(), triplets.cend());
		terzaghi.insertDisplacementTensionTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
}

TestCase("Displacement pressure term")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.alpha = 0.8;
	terzaghi.pressureGradient = {
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, {3.1, 6.7, 0.0}}, {1, {-1.2, 6.1, 0.0}} },
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, Eigen::Vector3d::Zero()} },
	};
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			{4, 0, -248.0/75.0},
			{4, 1, +1.28},
			{9, 0, -536.0/75.0},
			{9, 1, -488.0/75.0}
		};
		Eigen::SparseMatrix<double> matrix(terzaghi.linearSystemSize,terzaghi.linearSystemSize);
		matrix.setFromTriplets(triplets.cbegin(), triplets.cend());
		terzaghi.insertDisplacementPressureTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
}

TestCase("Displacement - stress prescribed")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.boundaries[0].stress << 0.0, +29.8, 0.0, 0.0, 0.0, 0.0;
	terzaghi.boundaries[1].stress << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	terzaghi.boundaries[2].stress << 0.0, 0.0, 0.0, +68.7, 0.0, 0.0;
	terzaghi.boundaries[3].stress << -35.7, 15.2, 0.0, 7.1, 0.0, 0.0;
	terzaghi.insertPrescribedStressInIndependent();
	Eigen::VectorXd independent = Eigen::VectorXd::Zero(terzaghi.linearSystemSize);
	independent[ 3] = +71.4;
	independent[ 8] = -14.2;
	independent[10] = -59.6;
	independent[11] = +137.4;
	check(terzaghi.linearSystem.independent==independent);
}

TestCase("Displacement - Neumann pressure boundary condition")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.alpha = 0.32;
		// top
		terzaghi.boundaries[0].isPressureDirichlet = false;
		terzaghi.boundaries[0].pressureGradient << 4.7, 2.4, 0.0;
		// bottom
		terzaghi.boundaries[1].isPressureDirichlet = false;
		terzaghi.boundaries[1].pressureGradient << -6.0, -8.3 , 0.0;
		// west
		terzaghi.boundaries[2].isPressureDirichlet = false;
		terzaghi.boundaries[2].pressureGradient << 7.0, -11.0, 0.0;
		// east
		terzaghi.boundaries[3].isPressureDirichlet = false;
		terzaghi.boundaries[3].pressureGradient << -2.7, 7.1, 0.0;
	terzaghi.insertDisplacementPressureNeumannBoundaryConditionToIndependent();
	Eigen::VectorXd independent = Eigen::VectorXd::Zero(terzaghi.linearSystemSize);
	independent[ 2] = - 1.28;
	independent[ 7] = - 664.0 / 375.0;
	independent[ 3] = - 0.576;
	independent[ 8] = + 568.0 / 375.0;
	independent[ 5] = + 376.0 / 375.0;
	independent[10] = + 0.512;
	independent[ 6] = + 112.0 / 75.0;
	independent[11] = - 176.0 / 75.0;
	check(terzaghi.linearSystem.independent==independent);
}

TestCase("Displacement - dirichlet boundary condition")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.fluidViscosity = 0.001;
	terzaghi.porosity = 0.19;
	terzaghi.alpha = 0.77777777777777;
	terzaghi.fluidCompressibility = 3.03030303030E-10;
	terzaghi.solidCompressibility = 2.77777777777E-11;
	terzaghi.permeability = 1.9E-15;
	terzaghi.timeInterval = 100;
	terzaghi.timeImplicitCoefficient = 1;
	terzaghi.shearModulus = 6.0E+9;
	terzaghi.poissonCoefficient = 0.2;
	section("matrix")
	{
		terzaghi.insertDisplacementTensionTermInMatrix();
		terzaghi.insertDisplacementDirichletBoundaryConditionToMatrix();
		auto checkIfBoundaryConditionWasApplied = [&](Component component, StaggeredElement2D* staggeredElement) -> void
		{
			const unsigned row = terzaghi.transformIndex(component, staggeredElement);
			for(auto& triplet: terzaghi.linearSystem.coefficients)
				if((triplet.row()==row) && (triplet.col()!=row))
					check(triplet.value()==0.0);
			for(auto& triplet: terzaghi.linearSystem.coefficients)
				if((triplet.row()==row) && (triplet.col()==row))
					check(triplet.value()==1.0);
		};
		checkIfBoundaryConditionWasApplied(Component::U, &(terzaghi.grid.staggeredElements[1]));
		checkIfBoundaryConditionWasApplied(Component::U, &(terzaghi.grid.staggeredElements[4]));
		checkIfBoundaryConditionWasApplied(Component::V, &(terzaghi.grid.staggeredElements[0]));
	}
	section("independent")
	{
		terzaghi.assemblyLinearSystemIndependent();
		auto checkIfIndependentIsNull = [&terzaghi](Component component, StaggeredElement2D* staggeredElement) -> void
		{
			const unsigned row = terzaghi.transformIndex(component, staggeredElement);
			check(terzaghi.linearSystem.independent[row]==0.0);
		};
		checkIfIndependentIsNull(Component::U, &(terzaghi.grid.staggeredElements[1]));
		checkIfIndependentIsNull(Component::U, &(terzaghi.grid.staggeredElements[4]));
		checkIfIndependentIsNull(Component::V, &(terzaghi.grid.staggeredElements[0]));
	}
}

TestCase("Displacement - pressure dirichlet boundary condition")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.alpha = 0.34;
	terzaghi.pressureGradient = {
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, Eigen::Vector3d::Zero()} },
		{ {0, {-4.8, 8.2, 0.0}}, {1, {1.8, -7.5, 0.0}} },
		{ {0, Eigen::Vector3d::Zero()} }
	};
	terzaghi.pressureGradientIndependent = {
		Eigen::Vector3d::Zero(),
		Eigen::Vector3d::Zero(),
		Eigen::Vector3d::Zero(),
		{-1.7, 5.5, 0.0},
		Eigen::Vector3d::Zero(),
	};
	terzaghi.boundaries[0].pressurePrescribedValue = 3.1;
	section("matrix")
	{
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			// staggered element 3
			{ 5, 0, 1.088},
			{10, 0, -697.0/375.0},
			{ 5, 1, -0.408},
			{10, 1, 1.7}
		};
		Eigen::SparseMatrix<double> matrix(terzaghi.linearSystemSize,terzaghi.linearSystemSize);
		matrix.setFromTriplets(triplets.cbegin(), triplets.cend());
		terzaghi.insertDisplacementPressureDirichletBoundaryConditionToMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		terzaghi.insertDisplacementPressureDirichletBoundaryConditionToIndependent();
		check(terzaghi.linearSystem.independent[ 5]==Approx(-1.194533333333));
		check(terzaghi.linearSystem.independent[10]==Approx(+3.864666666666));
	}
}