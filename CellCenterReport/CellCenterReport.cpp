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
#include <Utils/ExportConvergenceToCSV.hpp>

const std::string cartesianDirectory               = gridDirectory + std::string("cartesian/");
const std::string cartesianTriangleDirectory       = gridDirectory + std::string("cartesian_triangle/");
const std::string cartesianMixedDirectory          = gridDirectory + std::string("cartesian_triangle_and_quadrangle/");
const std::string unstructuredTriangleDirectory    = gridDirectory + std::string("unstructured_triangle/");
const std::string unstructuredQuadrangleDirectory  = gridDirectory + std::string("unstructured_quadrangle/");
const std::vector<std::string> cartesianFiles = {
	"03.cgns",
	"05.cgns",
	"10.cgns",
	"20.cgns"
};
const std::vector<std::string> unstructuredFiles = {
	"01.cgns",
	"02.cgns",
	"03.cgns",
	"04.cgns"
};
const std::string csvFileName = gridDirectory + "elementCenter_convergence.csv";

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

		// Compare
		Eigen::VectorXd difference = numericalSolution - analyticalSolution;
		double error = 0.0;
		for(auto element: problem.grid2D.elements)
		{
			const unsigned index = element->getIndex();
			error += element->getVolume() * difference[index] * difference[index];
		}
		error = std::sqrt(error);
		// double error = std::sqrt(difference.squaredNorm()/problem.grid2D.elements.size());
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getCharacteristicLength());
		std::cout << std::endl << "\t" << "erro: " << error << "  characteristic length:" << characteristicLength.back();
	}
	std::cout << std::endl;
	appendConvergenceToCSV(csvFileName, numericalError, characteristicLength);
	return RateOfConvergence(numericalError,characteristicLength);
};

int main()
{
	constexpr int width = 50;
	double orderOfConvergence;
	createConvergenceFile(csvFileName);
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