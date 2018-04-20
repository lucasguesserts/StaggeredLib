#include <array>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <CGNSFile/CGNSFile.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/Boundary.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

TestCase("Apply dirichlet boundary condition", "[SquareCavityHeatTransfer]")
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
	section("botton")
	{
		std::string boundaryName = "bottom boundary";
		DirichletBoundaryCondition dirichlet;
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue = 19.0;
		problem.applyBoundaryCondition(dirichlet);
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		matrix << 9.24, 0.0,
		          0.0 , 0.0;
		Eigen::VectorXd independent(numberOfElements);
		independent << 199.32, 0.0;
		check(problem.linearSystem.matrix==matrix);
		check(problem.linearSystem.independent==independent);
	}
}