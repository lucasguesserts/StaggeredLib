#include <array>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <iostream>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <CGNSFile/CGNSFile.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil_2.hpp>
#include <Grid/Boundary_2.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

TestCase("Compare numerical and analytical solution - mixed elements", "[SquareCavityHeatTransfer]")
{
	const std::string directory = CGNSFile::gridDirectory + std::string("SquareCavityHeatTransfer_AnalyticalTest/");
	const std::vector<std::string> meshFiles = {
		directory + "5.cgns",
		directory + "10.cgns",
		directory + "15.cgns"
	};
	std::vector<double> numericalError;
	for(auto& cgnsFileName: meshFiles)
	{
		std::cout << "File: " << cgnsFileName << std::endl;
		CGNSFile cgnsFile(cgnsFileName);
		GridData gridData(cgnsFile);
		SquareCavityHeatTransfer problem(gridData);
		problem.rho = 1;
		problem.cp = 1;
		problem.k = 1;
		problem.timeImplicitCoefficient = 1.0;
		problem.timeInterval = 1000;
		DirichletBoundaryCondition dirichlet;
		for(unsigned i=0 ; i<problem.oldTemperature.size() ; ++i)
			problem.temperature[i] = 0.0;
		std::string boundaryName;
			boundaryName = "bottom boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(auto& entry: dirichlet.prescribedValue)
				entry = 0.0;
			problem.dirichletBoundaries.push_back(dirichlet);
			boundaryName = "east boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(auto& entry: dirichlet.prescribedValue)
				entry = 0.0;
			problem.dirichletBoundaries.push_back(dirichlet);
			boundaryName = "top boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(unsigned i=0 ; i<dirichlet.staggeredTriangle.size() ; ++i)
			{
				Eigen::Vector3d centroid = dirichlet.staggeredTriangle[i]->getCentroid();
				const double x = centroid.coeff(0);
				const double y = centroid.coeff(1);
				dirichlet.prescribedValue[i] = std::sin(M_PI*x) * std::sinh(M_PI*y) / std::sinh(M_PI);
			}
			problem.dirichletBoundaries.push_back(dirichlet);
			boundaryName = "west boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(auto& entry: dirichlet.prescribedValue)
				entry = 0.0;
			problem.dirichletBoundaries.push_back(dirichlet);

		// Steady numerical solution
		unsigned iteration = 0;
		double error;
		constexpr double tolerance = 1.0e-8;
		do
		{
			std::cout << "\ttime step: " << iteration << std::endl;
			problem.oldTemperature = problem.temperature;
			problem.temperature = problem.nextTimeStep();
			error = (problem.temperature - problem.oldTemperature).norm();
			++iteration;
			std::cout << "\t\terror = " << error << std::endl;
		} while(error > tolerance);
		Eigen::VectorXd numericalSolution = problem.temperature;

		// Analytical
		Eigen::Matrix<double,Eigen::Dynamic,3> elementsCentroid;
		elementsCentroid.resize(problem.grid2D.elements.size(), Eigen::NoChange);
		for(auto element: problem.grid2D.elements)
		{
			Eigen::Vector3d centroid = element->getCentroid();
			elementsCentroid(element->getIndex(),0) = centroid(0);
			elementsCentroid(element->getIndex(),1) = centroid(1);
			elementsCentroid(element->getIndex(),2) = centroid(2);
		}
		Eigen::VectorXd analyticalSolution = SquareCavityHeatTransfer::computeAnalyticalSolution(elementsCentroid);

		error = (numericalSolution - analyticalSolution).norm() / numericalSolution.size();
		numericalError.push_back(error);
	}
	for(unsigned i=0 ; i<(numericalError.size()-1) ; ++i)
	{
		std::cout << "NumericalError[" << i << "] = " << numericalError[i] << std::endl;
		check(numericalError[i+1]<numericalError[i]);
	}
		std::cout << "NumericalError[" << (numericalError.size()-1) << "] = " << numericalError[(numericalError.size()-1)] << std::endl;
}

TestCase("Compare numerical and analytical solution - cartesian elements", "[SquareCavityHeatTransfer]")
{
	const std::string directory = CGNSFile::gridDirectory + std::string("SquareCavityHeatTransfer_AnalyticalTest_cartesian/");
	const std::vector<std::string> meshFiles = {
		directory + "5.cgns",
		directory + "10.cgns",
		directory + "15.cgns"
	};
	std::vector<double> numericalError;
	for(auto& cgnsFileName: meshFiles)
	{
		std::cout << "File: " << cgnsFileName << std::endl;
		CGNSFile cgnsFile(cgnsFileName);
		GridData gridData(cgnsFile);
		SquareCavityHeatTransfer problem(gridData);
		problem.rho = 1;
		problem.cp = 1;
		problem.k = 1;
		problem.timeImplicitCoefficient = 1.0;
		problem.timeInterval = 1000;
		DirichletBoundaryCondition dirichlet;
		for(unsigned i=0 ; i<problem.oldTemperature.size() ; ++i)
			problem.temperature[i] = 0.0;
		std::string boundaryName;
			boundaryName = "bottom boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(auto& entry: dirichlet.prescribedValue)
				entry = 0.0;
			problem.dirichletBoundaries.push_back(dirichlet);
			boundaryName = "east boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(auto& entry: dirichlet.prescribedValue)
				entry = 0.0;
			problem.dirichletBoundaries.push_back(dirichlet);
			boundaryName = "top boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(unsigned i=0 ; i<dirichlet.staggeredTriangle.size() ; ++i)
			{
				Eigen::Vector3d centroid = dirichlet.staggeredTriangle[i]->getCentroid();
				const double x = centroid.coeff(0);
				const double y = centroid.coeff(1);
				dirichlet.prescribedValue[i] = std::sin(M_PI*x) * std::sinh(M_PI*y) / std::sinh(M_PI);
			}
			problem.dirichletBoundaries.push_back(dirichlet);
			boundaryName = "west boundary";
			dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
			dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
			for(auto& entry: dirichlet.prescribedValue)
				entry = 0.0;
			problem.dirichletBoundaries.push_back(dirichlet);

		// Steady numerical solution
		unsigned iteration = 0;
		double error;
		constexpr double tolerance = 1.0e-8;
		do
		{
			std::cout << "\ttime step: " << iteration << std::endl;
			problem.oldTemperature = problem.temperature;
			problem.temperature = problem.nextTimeStep();
			error = (problem.temperature - problem.oldTemperature).norm();
			++iteration;
			std::cout << "\t\terror = " << error << std::endl;
		} while(error > tolerance);
		Eigen::VectorXd numericalSolution = problem.temperature;

		// Analytical
		Eigen::Matrix<double,Eigen::Dynamic,3> elementsCentroid;
		elementsCentroid.resize(problem.grid2D.elements.size(), Eigen::NoChange);
		for(auto element: problem.grid2D.elements)
		{
			Eigen::Vector3d centroid = element->getCentroid();
			elementsCentroid(element->getIndex(),0) = centroid(0);
			elementsCentroid(element->getIndex(),1) = centroid(1);
			elementsCentroid(element->getIndex(),2) = centroid(2);
		}
		Eigen::VectorXd analyticalSolution = SquareCavityHeatTransfer::computeAnalyticalSolution(elementsCentroid);

		error = (numericalSolution - analyticalSolution).norm();
		numericalError.push_back(error);
	}
	for(unsigned i=0 ; i<(numericalError.size()-1) ; ++i)
	{
		std::cout << "NumericalError[" << i << "] = " << numericalError[i] << std::endl;
		check(numericalError[i+1]<numericalError[i]);
	}
		std::cout << "NumericalError[" << (numericalError.size()-1) << "] = " << numericalError[(numericalError.size()-1)] << std::endl;
}