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
#include <CGNSFile/CGNSFile.hpp>
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
		FacetCenterHeatTransfer problem(gridData);
		DirichletBoundaryCondition dirichlet;
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
		problem.addDiffusiveTerm();
		problem.applyBoundaryConditions();
		std::cout << problem.linearSystem.matrix.determinant() << std::endl;
		Eigen::VectorXd numericalSolution = problem.linearSystem.solve();

		// Analytical
		Eigen::Matrix<double,Eigen::Dynamic,3> staggeredElementsCentroid;
		staggeredElementsCentroid.resize(problem.grid2D.staggeredElements.size(), Eigen::NoChange);
		for(auto staggeredElement: problem.grid2D.elements)
		{
			Eigen::Vector3d centroid = staggeredElement->getCentroid();
			staggeredElementsCentroid(staggeredElement->getIndex(),0) = centroid(0);
			staggeredElementsCentroid(staggeredElement->getIndex(),1) = centroid(1);
			staggeredElementsCentroid(staggeredElement->getIndex(),2) = centroid(2);
		}
		Eigen::VectorXd analyticalSolution = SquareCavityHeatTransfer::computeAnalyticalSolution(staggeredElementsCentroid);

		double error = (numericalSolution - analyticalSolution).norm();
		numericalError.push_back(error);
	}
	for(unsigned i=0 ; i<(numericalError.size()-1) ; ++i)
	{
		std::cout << "NumericalError[" << i << "] = " << numericalError[i] << std::endl;
		check(numericalError[i+1]<numericalError[i]);
	}
		std::cout << "NumericalError[" << (numericalError.size()-1) << "] = " << numericalError[(numericalError.size()-1)] << std::endl;
}