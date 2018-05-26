#include <array>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/Boundary.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>

TestCase("Insert dirichlet boundary condition to square cavity heat transfer", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	SquareCavityHeatTransfer problem(cgnsGridFileName);
	section("bottom - using lambda expression")
	{
		std::string boundaryName = "bottom boundary";
		auto dirichletFunction = [](Eigen::Vector3d centroid) -> double { return centroid.x()+3; };
		problem.insertDirichletBoundaryCondition(boundaryName, dirichletFunction);
		check(problem.dirichletBoundaries[0].staggeredTriangle.size()==1);
		check(problem.dirichletBoundaries[0].staggeredTriangle[0]==problem.grid2D.staggeredTriangles[0]);
		check(problem.dirichletBoundaries[0].prescribedValue[0]==4.0);
	}
	section("top - using prescribed value")
	{
		std::string boundaryName = "top boundary";
		constexpr double prescribedValue = 8.1;
		problem.insertDirichletBoundaryCondition(boundaryName, prescribedValue);
		check(problem.dirichletBoundaries[0].staggeredTriangle.size()==1);
		check(problem.dirichletBoundaries[0].staggeredTriangle[0]==problem.grid2D.staggeredTriangles[2]);
		check(problem.dirichletBoundaries[0].prescribedValue[0]==prescribedValue);
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
		constexpr double prescribedValue = 19.0;
		problem.insertDirichletBoundaryCondition(boundaryName,prescribedValue);
		section("matrix")
		{
			problem.applyBoundaryConditionsToMatrix();
			problem.linearSystem.computeLU();
			Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
			matrix << 9.24, 0.0,
			          0.0 , 0.0;
			check(problem.linearSystem.matrix==Eigen::SparseMatrix<double>(matrix.sparseView()));
		}
		section("independent")
		{
			problem.applyBoundaryConditionsToIndependent();
			Eigen::VectorXd independent(numberOfElements);
			independent << 199.32, 0.0;
			check(problem.linearSystem.independent==independent);
		}
	}
	section("east")
	{
		std::string boundaryName = "east boundary";
		constexpr double prescribedValue = 23.0;
		problem.insertDirichletBoundaryCondition(boundaryName,prescribedValue);
		section("matrix")
		{
			problem.applyBoundaryConditionsToMatrix();
			problem.linearSystem.computeLU();
			Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
			matrix << 9.24, 0.0,
			          0.0 , 0.0;
			check(problem.linearSystem.matrix==Eigen::SparseMatrix<double>(matrix.sparseView()));
		}
		section("independent")
		{
			problem.applyBoundaryConditionsToIndependent();
			Eigen::VectorXd independent(numberOfElements);
			independent << 252.12, 0.0;
			check(problem.linearSystem.independent==independent);
		}
	}
	section("west")
	{
		std::string boundaryName = "west boundary";
		constexpr double prescribedValue = 31.0;
		problem.insertDirichletBoundaryCondition(boundaryName,prescribedValue);
		section("matrix")
		{
			problem.applyBoundaryConditionsToMatrix();
			problem.linearSystem.computeLU();
			Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
			matrix << 0.0, 0.0,
			          0.0, 9.24;
			check(problem.linearSystem.matrix==Eigen::SparseMatrix<double>(matrix.sparseView()));
		}
		section("independent")
		{
			problem.applyBoundaryConditionsToIndependent();
			Eigen::VectorXd independent(numberOfElements);
			independent << 0.0, 341.88;
			check(problem.linearSystem.independent==independent);
		}
	}
	section("top")
	{
		std::string boundaryName = "top boundary";
		constexpr double prescribedValue = 29.0;
		problem.insertDirichletBoundaryCondition(boundaryName,prescribedValue);
		section("matrix")
		{
			problem.applyBoundaryConditionsToMatrix();
			problem.linearSystem.computeLU();
			Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
			matrix << 0.0, 0.0,
			          0.0, 9.24;
			check(problem.linearSystem.matrix==Eigen::SparseMatrix<double>(matrix.sparseView()));
		}
		section("independent")
		{
			problem.applyBoundaryConditionsToIndependent();
			Eigen::VectorXd independent(numberOfElements);
			independent << 0.0, 315.48;
			check(problem.linearSystem.independent==independent);
		}
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
	problem.insertDirichletBoundaryCondition("bottom boundary",19.0);
	problem.insertDirichletBoundaryCondition("east boundary",23.0);
	problem.insertDirichletBoundaryCondition("top boundary",29.0);
	problem.insertDirichletBoundaryCondition("west boundary",31.0);
	section("matrix")
	{
		problem.applyBoundaryConditionsToMatrix();
		problem.linearSystem.computeLU();
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		matrix << 18.48, 0.0,
				0.0  , 18.48;
		check(problem.linearSystem.matrix==Eigen::SparseMatrix<double>(matrix.sparseView()));
	}
	section("independent")
	{
		problem.applyBoundaryConditionsToIndependent();
		Eigen::VectorXd independent(numberOfElements);
		independent << 451.44, 657.36;
		check(problem.linearSystem.independent==independent);
	}
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
	DirichletBoundaryCondition dirichlet;
	std::string boundaryName;
	problem.insertDirichletBoundaryCondition("bottom boundary",19.0);
	problem.insertDirichletBoundaryCondition("east boundary",23.0);
	problem.insertDirichletBoundaryCondition("top boundary",29.0);
	problem.insertDirichletBoundaryCondition("west boundary",31.0);
	section("matrix")
	{
		problem.prepareMatrix();
		Eigen::MatrixXd matrix(numberOfElements,numberOfElements);
		matrix <<  42.03, -11.55,
		          -11.55,  42.03;
		check(problem.linearSystem.matrix==Eigen::SparseMatrix<double>(matrix.sparseView()));
	}
	section("independent")
	{
		problem.addAccumulationTermToIndependent();
		problem.addDiffusiveTermToIndependent();
		problem.applyBoundaryConditionsToIndependent();
		Eigen::VectorXd independent(numberOfElements);
		independent << 627.24, 841.56;
		check(problem.linearSystem.independent==independent);
	}
}