#include <array>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

TestCase("Add accumulation term", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = gridDirectory + "GridReaderTest_CGNS.cgns";
	SquareCavityHeatTransfer problem(cgnsGridFileName);
	problem.rho = 1;
	problem.cp = 1;
	problem.oldTemperature << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
	const unsigned size = problem.linearSystem.independent.size();
	problem.linearSystem.matrix += Eigen::MatrixXd::Ones(size,size);
	problem.addAccumulationTermToMatrix();
	problem.addAccumulationTermToIndependent();
	problem.linearSystem.matrix -= Eigen::MatrixXd::Ones(size,size);
	section("independent")
	{
		const unsigned numberOfElements = problem.grid2D.elements.size();
		Eigen::VectorXd independent;
		independent.resize(numberOfElements);
		independent << 2.250 * 0.0,
					   2.250 * 1.0,
		               1.125 * 3.0,
					   1.125 * 2.0,
					   1.125 * 5.0,
					   1.125 * 4.0;
		check(problem.linearSystem.independent==independent);
	}
	section("matrix")
	{
		const unsigned numberOfElements = problem.grid2D.elements.size();
		Eigen::MatrixXd matrix;
		matrix.resize(numberOfElements,numberOfElements);
		matrix << 2.250, 0.000, 0.000, 0.000, 0.000, 0.000,
		          0.000, 2.250, 0.000, 0.000, 0.000, 0.000,
		          0.000, 0.000, 1.125, 0.000, 0.000, 0.000,
		          0.000, 0.000, 0.000, 1.125, 0.000, 0.000,
		          0.000, 0.000, 0.000, 0.000, 1.125, 0.000,
		          0.000, 0.000, 0.000, 0.000, 0.000, 1.125;
		check(problem.linearSystem.matrix==matrix);
	}
	section("solve")
	{
		const unsigned numberOfElements = problem.grid2D.elements.size();
		Eigen::VectorXd temperature;
		temperature.resize(numberOfElements);
		temperature << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
		problem.temperature = problem.linearSystem.solve();
		check(problem.temperature==temperature);
	}
}

TestCase("Add diffusive term for one staggered quadrangle", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	SquareCavityHeatTransfer problem(cgnsGridFileName);
	const unsigned numberOfElements = problem.grid2D.elements.size();
	problem.rho = 2;
	problem.cp = 3;
	problem.k = 5;
	problem.timeImplicitCoefficient = 0.7;
	problem.timeInterval = 1.1;
	problem.oldTemperature << 13, 17;
	section("matrix")
	{
		problem.addAccumulationTermToMatrix();
		problem.addDiffusiveTermToMatrix();
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		Eigen::MatrixXd(numberOfElements,numberOfElements);
		matrix <<  23.55, -11.55,
				-11.55,  23.55;
		check(problem.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		problem.addAccumulationTermToIndependent();
		problem.addDiffusiveTermToIndependent();
		Eigen::VectorXd independent(numberOfElements);
		independent << 175.8, 184.2;
		check(problem.linearSystem.independent==independent);
	}
}