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
#include <LinearSystem/EigenLinearSystem.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>

TestCase("Facet center method - compare numerical and analytical solution", "[FacetCenterHeatTransfer]")
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
	// check(rateOfConvergence.converge());
	// check(rateOfConvergence.order>0.5);
}

TestCase("Facet center method - four_quadrangles", "[FacetCenterHeatTransfer]")
{
	const std::string cgnsFileName = gridDirectory + "SquareCavityHeatTransfer_AnalyticalTest/15.cgns";
	FacetCenterHeatTransfer problem(cgnsFileName);
	problem.insertDirichletBoundaryCondition("east boundary", 1.0);
	problem.insertDirichletBoundaryCondition("west boundary", 0.0);

		// Steady numerical solution
		problem.addDiffusiveTerm();
		problem.applyBoundaryConditions();

		// std::cout << std::endl << "Linear system determinant: " << std::endl << problem.linearSystem.matrix.determinant() << std::endl << std::endl;
		// std::cout << std::endl << "Linear system matrix: " << std::endl << problem.linearSystem.matrix << std::endl << std::endl;
		// std::cout << std::endl << "Linear system independent: " << std::endl << problem.linearSystem.independent << std::endl << std::endl;

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
		auto computeAnalytical = [](Eigen::Matrix<double,Eigen::Dynamic,3> staggeredElementsCentroid) -> Eigen::VectorXd
		{
			Eigen::VectorXd analytical(staggeredElementsCentroid.rows());
			for(unsigned i=0 ; i<staggeredElementsCentroid.rows() ; ++i)
			{
				analytical(i) = staggeredElementsCentroid(i,0);
			}
			return analytical;
		};
		Eigen::VectorXd analyticalSolution = computeAnalytical(staggeredElementsCentroid);

		double error = (numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>();

		// std::cout << std::endl << "Numerical: " << std::endl << numericalSolution << std::endl << std::endl;
		// std::cout << std::endl << "Analytical: " << std::endl << analyticalSolution << std::endl << std::endl;
		std::cout << std::endl << "Error (norm inf): " << error << std::endl << std::endl;

		// const unsigned rows = problem.linearSystem.matrix.rows();
		// std::cout << std::endl << "number of staggered elements \n" << rows << std::endl;
		// std::cout << std::endl << "staggered element 10 line:: \n" << problem.linearSystem.matrix.block(10,0,1,rows) << std::endl << std::endl;

		// check(numericalSolution==analyticalSolution);

}