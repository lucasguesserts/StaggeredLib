#include <array>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/Boundary.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

TestCase("Insert dirichlet boundary condition to square cavity heat transfer", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	SquareCavityHeatTransfer problem(cgnsGridFileName);
	section("bottom")
	{
		std::string boundaryName = "bottom boundary";
		auto dirichletFunction = [](Eigen::Vector3d centroid) -> double { return centroid.x()+3; };
		problem.insertDirichletBoundaryCondition(boundaryName, dirichletFunction);
		check(problem.dirichletBoundaries[0].staggeredTriangle.size()==1);
		check(problem.dirichletBoundaries[0].staggeredTriangle[0]==problem.grid2D.staggeredTriangles[0]);
		check(problem.dirichletBoundaries[0].prescribedValue[0]==4.0);
	}
}

TestCase("Apply dirichlet boundary condition one boundary at time", "[SquareCavityHeatTransfer]")
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
	section("bottom")
	{
		std::string boundaryName = "bottom boundary";
		DirichletBoundaryCondition dirichlet;
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 19.0;
		problem.applyBoundaryCondition(dirichlet);
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		matrix << 9.24, 0.0,
		          0.0 , 0.0;
		Eigen::VectorXd independent(numberOfElements);
		independent << 199.32, 0.0;
		check(problem.linearSystem.matrix==matrix);
		check(problem.linearSystem.independent==independent);
	}
	section("east")
	{
		std::string boundaryName = "east boundary";
		DirichletBoundaryCondition dirichlet;
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 23.0;
		problem.applyBoundaryCondition(dirichlet);
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		matrix << 9.24, 0.0,
		          0.0 , 0.0;
		Eigen::VectorXd independent(numberOfElements);
		independent << 252.12, 0.0;
		check(problem.linearSystem.matrix==matrix);
		check(problem.linearSystem.independent==independent);
	}
	section("west")
	{
		std::string boundaryName = "west boundary";
		DirichletBoundaryCondition dirichlet;
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 31.0;
		problem.applyBoundaryCondition(dirichlet);
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		matrix << 0.0, 0.0,
		          0.0, 9.24;
		Eigen::VectorXd independent(numberOfElements);
		independent << 0.0, 341.88;
		check(problem.linearSystem.matrix==matrix);
		check(problem.linearSystem.independent==independent);
	}
	section("top")
	{
		std::string boundaryName = "top boundary";
		DirichletBoundaryCondition dirichlet;
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 29.0;
		problem.applyBoundaryCondition(dirichlet);
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		matrix << 0.0, 0.0,
		          0.0, 9.24;
		Eigen::VectorXd independent(numberOfElements);
		independent << 0.0, 315.48;
		check(problem.linearSystem.matrix==matrix);
		check(problem.linearSystem.independent==independent);
	}
}

TestCase("Apply dirichlet boundary condition - all boundaries", "[SquareCavityHeatTransfer]")
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
	DirichletBoundaryCondition dirichlet;
	std::string boundaryName;
		boundaryName = "bottom boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 19.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "east boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 23.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "top boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 29.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "west boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 31.0;
		problem.dirichletBoundaries.push_back(dirichlet);
	problem.applyBoundaryConditions();
	Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
	matrix << 18.48, 0.0,
	          0.0  , 18.48;
	Eigen::VectorXd independent(numberOfElements);
	independent << 451.44, 657.36;
	check(problem.linearSystem.matrix==matrix);
	check(problem.linearSystem.independent==independent);
}

TestCase("Complete heat transfer with dirichlet boundary conditions", "[SquareCavityHeatTransfer]")
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
	problem.addAccumulationTerm();
	problem.addDiffusiveTerm();
	DirichletBoundaryCondition dirichlet;
	std::string boundaryName;
		boundaryName = "bottom boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 19.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "east boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 23.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "top boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 29.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "west boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 31.0;
		problem.dirichletBoundaries.push_back(dirichlet);
	problem.applyBoundaryConditions();
	Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
	matrix <<  42.03, -11.55,
	          -11.55,  42.03;
	Eigen::VectorXd independent(numberOfElements);
	independent << 627.24, 841.56;
	check(problem.linearSystem.matrix==matrix);
	check(problem.linearSystem.independent==independent);
}