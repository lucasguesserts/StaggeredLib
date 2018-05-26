#include <array>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/filesystem.hpp>
#include <stdexcept>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Utils/RateOfConvergence.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/Boundary.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>

TestCase("Compare numerical and analytical solution - mixed elements", "[SquareCavityHeatTransfer]")
{
	const std::string directory = gridDirectory + std::string("SquareCavityHeatTransfer_AnalyticalTest/");
	const std::vector<std::string> meshFiles = {
		directory + "5.cgns",
		directory + "10.cgns",
		directory + "15.cgns"
	};
	std::vector<double> numericalError, characteristicLength;
	for(auto& cgnsFileName: meshFiles)
	{
		SquareCavityHeatTransfer problem(cgnsFileName);
		problem.rho = 1;
		problem.cp = 1;
		problem.k = 1;
		problem.timeImplicitCoefficient = 1.0;
		problem.timeInterval = 1000;
		for(unsigned i=0 ; i<problem.temperature.size() ; ++i)
			problem.temperature[i] = 0.0;
		problem.insertDirichletBoundaryCondition("bottom boundary",0.0);
		problem.insertDirichletBoundaryCondition("east boundary",0.0);
		problem.insertDirichletBoundaryCondition("west boundary",0.0);
		problem.insertDirichletBoundaryCondition("top boundary",
			[](Eigen::Vector3d centroid) -> double
			{ return std::sin(M_PI*centroid.x()) * std::sinh(M_PI*centroid.y()) / std::sinh(M_PI); });

		// Steady numerical solution
		unsigned iteration = 0;
		double error;
		constexpr double tolerance = 1.0e-8;
		problem.prepareMatrix();
		do
		{
			problem.oldTemperature = problem.temperature;
			problem.temperature = problem.nextTimeStep();
			error = (problem.temperature - problem.oldTemperature).lpNorm<Eigen::Infinity>();
			++iteration;
		} while(error > tolerance);
		Eigen::VectorXd numericalSolution = problem.temperature;

		// error
		Eigen::VectorXd analyticalSolution = problem.computeAnalyticalSolution();
		error = (numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>();
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getCharacteristicLength());
	}
	RateOfConvergence rateOfConvergence(numericalError,characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.order>0.5);
}

TestCase("Compare numerical and analytical solution - cartesian elements", "[SquareCavityHeatTransfer]")
{
	const std::string directory = gridDirectory + std::string("SquareCavityHeatTransfer_AnalyticalTest_cartesian/");
	const std::vector<std::string> meshFiles = {
		directory + "5.cgns",
		directory + "10.cgns",
		directory + "15.cgns"
	};
	std::vector<double> numericalError, characteristicLength;
	for(auto& cgnsFileName: meshFiles)
	{
		SquareCavityHeatTransfer problem(cgnsFileName);
		problem.rho = 1;
		problem.cp = 1;
		problem.k = 1;
		problem.timeImplicitCoefficient = 1.0;
		problem.timeInterval = 1000;
		for(unsigned i=0 ; i<problem.oldTemperature.size() ; ++i)
			problem.temperature[i] = 0.0;
		problem.insertDirichletBoundaryCondition("bottom boundary",0.0);
		problem.insertDirichletBoundaryCondition("east boundary",0.0);
		problem.insertDirichletBoundaryCondition("west boundary",0.0);
		problem.insertDirichletBoundaryCondition("top boundary",
			[](Eigen::Vector3d centroid) -> double
			{ return std::sin(M_PI*centroid.x()) * std::sinh(M_PI*centroid.y()) / std::sinh(M_PI); });

		// Steady numerical solution
		unsigned iteration = 0;
		double error;
		constexpr double tolerance = 1.0e-8;
		problem.prepareMatrix();
		do
		{
			problem.oldTemperature = problem.temperature;
			problem.temperature = problem.nextTimeStep();
			error = (problem.temperature - problem.oldTemperature).lpNorm<Eigen::Infinity>();
		} while(error > tolerance);
		Eigen::VectorXd numericalSolution = problem.temperature;

		// error
		Eigen::VectorXd analyticalSolution = problem.computeAnalyticalSolution();
		error = (numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>();
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getCharacteristicLength());
	}
	RateOfConvergence rateOfConvergence(numericalError,characteristicLength);
	check(rateOfConvergence.isLinear());
}