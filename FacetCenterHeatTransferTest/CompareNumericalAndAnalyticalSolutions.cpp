#include <array>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>

#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Utils/RateOfConvergence.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/Boundary.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <Grid/Grid2DWithStaggeredElementsExport.hpp>
#include <CgnsInterface/CgnsWriter.hpp>
#include <CgnsInterface/CgnsReader/CgnsReader2D.hpp>

TestCase("Facet center method - compare numerical and analytical solution - regular grids", "[FacetCenterHeatTransfer]")
{
	const std::string directory = gridDirectory + std::string("SquareCavityHeatTransfer_AnalyticalTest/");
	const std::string outputDirectory = gridDirectory + std::string("output/");
	const std::vector<std::string> meshFiles = {
		directory + "3.cgns",
		directory + "5.cgns",
		directory + "10.cgns",
		directory + "15.cgns"
	};
	const std::vector<std::string> resultFiles = {
		outputDirectory + "facet_center_3.cgns",
		outputDirectory + "facet_center_5.cgns",
		outputDirectory + "facet_center_10.cgns",
		outputDirectory + "facet_center_15.cgns"
	};
	std::vector<double> numericalError, characteristicLength;
	for(unsigned count=0 ; count<meshFiles.size() ; ++count)
	{
		std::string cgnsFileName = meshFiles[count];
		std::string resultFileName = resultFiles[count];
		FacetCenterHeatTransfer problem(cgnsFileName);

		problem.insertDirichletBoundaryCondition("bottom boundary",0.0);
		problem.insertDirichletBoundaryCondition("east boundary",0.0);
		problem.insertDirichletBoundaryCondition("west boundary",0.0);
		problem.insertDirichletBoundaryCondition("top boundary",
			[](Eigen::Vector3d centroid) -> double
			{ return std::sin(M_PI*centroid.x()) * std::sinh(M_PI*centroid.y()) / std::sinh(M_PI); });

		// Steady numerical solution
		problem.addDiffusiveTerm();
		problem.applyBoundaryConditions();
		problem.linearSystem.computeLU();
		Eigen::VectorXd numericalSolution = problem.linearSystem.solve();

		// Analytical
		Eigen::Matrix<double,Eigen::Dynamic,3> staggeredElementsCentroid;
		staggeredElementsCentroid.resize(problem.grid2D.staggeredElements.size(), Eigen::NoChange);
		for(auto staggeredElement: problem.grid2D.staggeredElements)
		{
			Eigen::Vector3d centroid = staggeredElement.getCentroid();
			staggeredElementsCentroid(staggeredElement.getIndex(),0) = centroid(0);
			staggeredElementsCentroid(staggeredElement.getIndex(),1) = centroid(1);
			staggeredElementsCentroid(staggeredElement.getIndex(),2) = centroid(2);
		}
		Eigen::VectorXd analyticalSolution = SquareCavityHeatTransfer::computeAnalyticalSolution(staggeredElementsCentroid);

		Eigen::VectorXd difference = numericalSolution - analyticalSolution;
		double error = difference.lpNorm<Eigen::Infinity>();
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getStaggeredCharacteristicLength());

		// Export
		Grid2DWithStaggeredElementsExport::cgns(resultFileName, problem.grid2D);
		CgnsWriter cgnsWriter(resultFileName, "CellCenter");
		cgnsWriter.writePermanentSolution("steadySolution");
		cgnsWriter.writePermanentField("Numerical_Temperature", std::vector<double>(&numericalSolution[0], &numericalSolution[0] + numericalSolution.size()));
		cgnsWriter.writePermanentField("Analytical_Temperature", std::vector<double>(&analyticalSolution[0], &analyticalSolution[0] + analyticalSolution.size()));
		cgnsWriter.writePermanentField("error", std::vector<double>(&difference[0], &difference[0] + difference.size()));
	}
	RateOfConvergence rateOfConvergence(numericalError,characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.order>0.2);
}

TestCase("Facet center method - compare numerical and analytical solution - irregular grids", "[FacetCenterHeatTransfer]")
{
	const std::string directory = gridDirectory + std::string("gmsh/");
	const std::string outputDirectory = gridDirectory + std::string("output/");
	const std::vector<std::string> meshFiles = {
		directory + "01.cgns",
		directory + "02.cgns",
		directory + "05.cgns"
	};
	const std::vector<std::string> resultFiles = {
		outputDirectory + "facet_center_gmsh_1.cgns",
		outputDirectory + "facet_center_gmsh_2.cgns",
		outputDirectory + "facet_center_gmsh_5.cgns"
	};
	std::vector<double> numericalError, characteristicLength;
	for(unsigned count=0 ; count<meshFiles.size() ; ++count)
	{
		std::string cgnsFileName = meshFiles[count];
		std::string resultFileName = resultFiles[count];
		FacetCenterHeatTransfer problem(cgnsFileName);

		problem.insertDirichletBoundaryCondition("South",0.0);
		problem.insertDirichletBoundaryCondition("East",0.0);
		problem.insertDirichletBoundaryCondition("West",0.0);
		problem.insertDirichletBoundaryCondition("North",
			[](Eigen::Vector3d centroid) -> double
			{ return std::sin(M_PI*centroid.x()) * std::sinh(M_PI*centroid.y()) / std::sinh(M_PI); });

		// Steady numerical solution
		problem.addDiffusiveTerm();
		problem.applyBoundaryConditions();
		problem.linearSystem.computeLU();
		Eigen::VectorXd numericalSolution = problem.linearSystem.solve();

		// Analytical
		Eigen::Matrix<double,Eigen::Dynamic,3> staggeredElementsCentroid;
		staggeredElementsCentroid.resize(problem.grid2D.staggeredElements.size(), Eigen::NoChange);
		for(auto staggeredElement: problem.grid2D.staggeredElements)
		{
			Eigen::Vector3d centroid = staggeredElement.getCentroid();
			staggeredElementsCentroid(staggeredElement.getIndex(),0) = centroid(0);
			staggeredElementsCentroid(staggeredElement.getIndex(),1) = centroid(1);
			staggeredElementsCentroid(staggeredElement.getIndex(),2) = centroid(2);
		}
		Eigen::VectorXd analyticalSolution = SquareCavityHeatTransfer::computeAnalyticalSolution(staggeredElementsCentroid);

		Eigen::VectorXd difference = numericalSolution - analyticalSolution;
		double error = difference.lpNorm<Eigen::Infinity>();
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getStaggeredCharacteristicLength());

		// Export
		Grid2DWithStaggeredElementsExport::cgns(resultFileName, problem.grid2D);
		CgnsWriter cgnsWriter(resultFileName, "CellCenter");
		cgnsWriter.writePermanentSolution("steadySolution");
		cgnsWriter.writePermanentField("Numerical_Temperature", std::vector<double>(&numericalSolution[0], &numericalSolution[0] + numericalSolution.size()));
		cgnsWriter.writePermanentField("Analytical_Temperature", std::vector<double>(&analyticalSolution[0], &analyticalSolution[0] + analyticalSolution.size()));
		cgnsWriter.writePermanentField("error", std::vector<double>(&difference[0], &difference[0] + difference.size()));
	}
	RateOfConvergence rateOfConvergence(numericalError,characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.order>1.0);
}