#include <array>

#include <boost/filesystem.hpp>

#include <Utils/Test.hpp>
#include <CGNSFile/CGNSFile.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

TestCase("Analytical solution", "[SquareCavityHeatTransfer]")
{
	const unsigned numberOfPoints = 4;
	Eigen::Matrix<double,4,3> coordinates;
	coordinates << 0.1, 0.1, 0.0,
	               0.5, 0.4, 0.0,
	               0.9, 1.0, 0.0,
	               0.7, 3.0, 0.0;
	Eigen::Vector4d solution, correctSolution;
	correctSolution << 0.008545119856,
	                   0.1397977728,
	                   0.3090169944,
	                   434.0323775;
	solution = SquareCavityHeatTransfer::computeAnalyticalSolution(coordinates);
	for(unsigned pointIndex=0 ; pointIndex<numberOfPoints ; ++pointIndex)
	{
		check(solution[pointIndex]==Approx(correctSolution[pointIndex]));
	}
}

TestCase("Linear system build", "[Eigen][EigenSolver]")
{
	const unsigned linearSystemSize = 3;
	EigenLinearSystem linearSystem;
	linearSystem.setSize(linearSystemSize);
	linearSystem.matrix << 0.462026696884809, 0.289318074606471, 0.418654423757951,
	                       0.111618554500326, 0.846885511973542, 0.805839771614926,
	                       0.664857365801401, 0.240344399557209, 0.348561451918039;
	linearSystem.independent << 0.263941635488370, 0.246987790661933, 0.706521526629329;
	Eigen::VectorXd solution;
	solution.resize(linearSystemSize);
	solution << 1.74452683050882, 3.77822561620858, -3.90581157526298;
	Eigen::VectorXd linearSystemSolution = linearSystem.solve();
	for(unsigned index=0 ; index<linearSystemSize ; ++index)
		check(solution[index]==Approx(linearSystemSolution[index]));
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
	problem.addAccumulationTerm();
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
	for(unsigned line=0 ; line<size ; ++line)
		linearSystem.addScalarStencil(line,scalarStencil[line]);
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size,size);
	matrix << scalarStencil[0][0], scalarStencil[0][1], scalarStencil[0][2],
	          scalarStencil[1][0], scalarStencil[1][1], scalarStencil[1][2],
	          scalarStencil[2][0], scalarStencil[2][1], scalarStencil[2][2];
	check(linearSystem.matrix==matrix);
}

TestCase("Add diffusive term for one staggered quadrangle", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DInverseDistanceStencil grid(gridData);
	const unsigned staggeredQuadrangleIndex = 0;
	grid.staggeredQuadrangles.emplace_back(StaggeredQuadrangle(staggeredQuadrangleIndex,grid.vertices[0],grid.elements[0],grid.vertices[3],grid.elements[1])); // TODO: remove when grid with staggered elements is complete.
}
