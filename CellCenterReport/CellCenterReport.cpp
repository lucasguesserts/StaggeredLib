#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>

#include <Utils/RateOfConvergence.hpp>
#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <Grid/Grid2DWithStaggeredElementsExport.hpp>
#include <CgnsInterface/CgnsReader/CgnsReader2D.hpp>

const std::string cartesianDirectory               = gridDirectory + std::string("cartesian/");
const std::string cartesianTriangleDirectory       = gridDirectory + std::string("cartesian_triangle/");
const std::string cartesianMixedDirectory          = gridDirectory + std::string("cartesian_triangle_and_quadrangle/");
const std::string unstructuredTriangleDirectory    = gridDirectory + std::string("unstructured_triangle/");
const std::string unstructuredQuadrangleDirectory  = gridDirectory + std::string("unstructured_quadrangle/");
const std::vector<std::string> cartesianFiles = {
	"03.cgns",
	"05.cgns",
	"10.cgns",
	"20.cgns",
	"50.cgns"
};
const std::vector<std::string> unstructuredFiles = {
	"01.cgns",
	"02.cgns",
	"03.cgns"
};

auto analiseConvergence = [](const std::string& directory, const std::vector<std::string>& fileNames) -> RateOfConvergence
{
	std::vector<double> numericalError, characteristicLength;
	for(auto& fileName: fileNames)
	{
		SquareCavityHeatTransfer problem(directory + fileName);

		problem.rho = 1;
		problem.cp = 1;
		problem.k = 1;
		problem.timeImplicitCoefficient = 1.0;
		problem.timeInterval = 1000;
		problem.insertDirichletBoundaryCondition("bottom boundary",0.0);
		problem.insertDirichletBoundaryCondition("east boundary",0.0);
		problem.insertDirichletBoundaryCondition("west boundary",0.0);
		problem.insertDirichletBoundaryCondition("top boundary",
			[](Eigen::Vector3d centroid) -> double
			{ return std::sin(M_PI*centroid.x()) * std::sinh(M_PI*centroid.y()) / std::sinh(M_PI); });

		// Steady numerical solution
		unsigned iteration = 0;
		double convergenceError;
		constexpr double tolerance = 1.0e-8;
		problem.prepareMatrix();
		do
		{
			problem.oldTemperature = problem.temperature;
			problem.temperature = problem.nextTimeStep();
			convergenceError = (problem.temperature - problem.oldTemperature).lpNorm<Eigen::Infinity>();
			++iteration;
		} while(convergenceError > tolerance);
		Eigen::VectorXd numericalSolution = problem.temperature;

		// Analytical
		Eigen::VectorXd analyticalSolution = problem.computeAnalyticalSolution();

		double error = (numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>();
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getStaggeredCharacteristicLength());
		std::cout << std::endl << "\t" << "erro: " << error << "  characteristic length:" << characteristicLength.back();
	}
	std::cout << std::endl;
	return RateOfConvergence(numericalError,characteristicLength);
};

int main()
{
	constexpr int width = 50;
	double orderOfConvergence;
	std::cout << std::setw(width) << std::left << "Mesh type" << "Order of convergence" << std::endl;

	orderOfConvergence = analiseConvergence(cartesianDirectory,cartesianFiles).order;
	std::cout << std::setw(width) << std::left << "Cartesian quadrangles" << orderOfConvergence << std::endl;

	orderOfConvergence = analiseConvergence(cartesianTriangleDirectory,cartesianFiles).order;
	std::cout << std::setw(width) << std::left << "Cartesian triangles" << orderOfConvergence << std::endl;

	orderOfConvergence = analiseConvergence(cartesianMixedDirectory,cartesianFiles).order;
	std::cout << std::setw(width) << std::left << "Cartesian mixed" << orderOfConvergence << std::endl;

	orderOfConvergence = analiseConvergence(unstructuredTriangleDirectory,unstructuredFiles).order;
	std::cout << std::setw(width) << std::left << "unstructured triangles" << orderOfConvergence << std::endl;

	orderOfConvergence = analiseConvergence(unstructuredQuadrangleDirectory,unstructuredFiles).order;
	std::cout << std::setw(width) << std::left << "unstructured quadrangles" << orderOfConvergence << std::endl;

	return 0;
}