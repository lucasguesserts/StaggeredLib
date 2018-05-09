#include <array>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <CGNSFile/CGNSFile.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

TestCase("Linear system build", "[Eigen][EigenSolver]")
{
	const unsigned linearSystemSize = 3;
	EigenLinearSystem linearSystem;
	linearSystem.setSize(linearSystemSize);
	linearSystem.matrix << 0.462026696884809, 0.289318074606471, 0.418654423757951,
	                       0.111618554500326, 0.846885511973542, 0.805839771614926,
	                       0.664857365801401, 0.240344399557209, 0.348561451918039;
	linearSystem.independent << 0.263941635488370, 0.246987790661933, 0.706521526629329;
	Eigen::VectorXd solution(linearSystemSize);
	solution << 1.74452683050882, 3.77822561620858, -3.90581157526298;
	Eigen::VectorXd linearSystemSolution = linearSystem.solve();
	check(solution==linearSystemSolution);
}

TestCase("Add accumulation term", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	SquareCavityHeatTransfer problem(gridData);
	problem.rho = 1;
	problem.cp = 1;
	problem.oldTemperature << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
	const unsigned size = problem.linearSystem.independent.size();
	problem.linearSystem.matrix += Eigen::MatrixXd::Ones(size,size);
	problem.addAccumulationTerm();
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

TestCase("Add scalar stencil to eigen linear system", "[ScalarStencil][EigenLinearSystem]")
{
	const unsigned size = 3;
	std::vector<ScalarStencil> scalarStencil = {
		{{0, 0.6}, {1, 1.9}, {2, 7.6}},
		{{2, 8.9}, {0, 5.2}, {1, 8.4}},
		{{2, 3.7}, {1, 2.0}}
	};
	EigenLinearSystem linearSystem;
	linearSystem.setSize(size);
	linearSystem.matrix = Eigen::MatrixXd::Ones(size,size);
	for(unsigned line=0 ; line<size ; ++line)
		linearSystem.addScalarStencil(line,scalarStencil[line]);
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size,size);
	matrix << 1+scalarStencil[0][0], 1+scalarStencil[0][1], 1+scalarStencil[0][2],
	          1+scalarStencil[1][0], 1+scalarStencil[1][1], 1+scalarStencil[1][2],
	          1+scalarStencil[2][0], 1+scalarStencil[2][1], 1+scalarStencil[2][2];
	check(linearSystem.matrix==matrix);
}

TestCase("Add diffusive term for one staggered quadrangle", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	SquareCavityHeatTransfer problem(gridData);
	const unsigned numberOfElements = problem.grid2D.elements.size();
	problem.rho = 2;
	problem.cp = 3;
	problem.k = 5;
	problem.timeImplicitCoefficient = 0.7;
	problem.timeInterval = 1.1;
	problem.oldTemperature << 13, 17;
	Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
	problem.addAccumulationTerm();
	problem.addDiffusiveTerm();
	Eigen::MatrixXd(numberOfElements,numberOfElements);
	matrix <<  23.55, -11.55,
	          -11.55,  23.55;
	Eigen::VectorXd independent(numberOfElements);
	independent << 175.8, 184.2;
	check(problem.linearSystem.matrix==matrix);
	check(problem.linearSystem.independent==independent);
}

TestCase("Add diffusive term all staggered elements", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	SquareCavityHeatTransfer problem(gridData);
	const unsigned numberOfElements = problem.grid2D.elements.size();
	problem.rho = 2;
	problem.cp = 3;
	problem.k = 5;
	problem.timeImplicitCoefficient = 0.7;
	problem.timeInterval = 1.1;
	problem.oldTemperature << 13, 17;
	// accumulation term
	problem.addAccumulationTerm();
	Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
	matrix << 12.0, 0.00,
	          0.00, 12.0;
	check(problem.linearSystem.matrix==matrix);
	// diffusive term
	for(auto& staggeredQuadrangle: problem.grid2D.staggeredQuadrangles)
		problem.addDiffusiveTerm(staggeredQuadrangle);
	Eigen::VectorXd independent(numberOfElements);
	matrix << 23.55, -11.55,
	          -11.55, 23.55;
	independent << 175.8, 184.2;
	check(problem.linearSystem.matrix==matrix);
	check(problem.linearSystem.independent==independent);
}