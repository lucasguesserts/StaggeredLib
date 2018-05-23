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
#include <PetersFacetCenterHeatTransfer/PetersFacetCenterHeatTransfer.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <Grid/Grid2DWithStaggeredElementsExport.hpp>
#include <CgnsInterface/CgnsWriter.hpp>

TestCase("Facet center method - compare numerical and analytical solution", "[PetersFacetCenterHeatTransfer]")
{
	const std::string directory = gridDirectory + std::string("SquareCavityHeatTransfer_AnalyticalTest/");
	const std::vector<std::string> meshFiles = {
		directory + "5.cgns",
		directory + "10.cgns",
		directory + "15.cgns"
	};
	const std::vector<std::string> resultFiles = {
		directory + "result_5.cgns",
		directory + "result_10.cgns",
		directory + "result_15.cgns"
	};
	std::vector<double> numericalError, characteristicLength;
	for(unsigned count=0 ; count<meshFiles.size() ; ++count)
	{
		std::string cgnsFileName = meshFiles[count];
		std::string resultFileName = resultFiles[count];
		PetersFacetCenterHeatTransfer problem(cgnsFileName);

		problem.insertDirichletBoundaryCondition("bottom boundary",0.0);
		problem.insertDirichletBoundaryCondition("east boundary",0.0);
		problem.insertDirichletBoundaryCondition("west boundary",0.0);
		problem.insertDirichletBoundaryCondition("top boundary",
			[](Eigen::Vector3d centroid) -> double
			{ return std::sin(M_PI*centroid.x()) * std::sinh(M_PI*centroid.y()) / std::sinh(M_PI); });

		// Steady numerical solution
		problem.addDiffusiveTerm();
		problem.applyBoundaryConditions();
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

		double error = (numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>();
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getStaggeredCharacteristicLength());
	}
	RateOfConvergence rateOfConvergence(numericalError,characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.order>0.1);
}