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

#include <Grid/Grid2DExport.hpp>
#include <Grid/Grid2DWithStaggeredElementsExport.hpp>
#include <CgnsInterface/CgnsWriter.hpp>
#include <CgnsInterface/CgnsReader/CgnsReader2D.hpp>




const std::string unstructuredQuadrangleDirectory = std::string("/home/herminio/cgns_learning/output/");
const std::string unstructuredFile = std::string("UnstructuredGrid2D.cgns");

const std::string csvFileName = gridDirectory + "elementCenter_convergence.csv";



int main()
{
	constexpr int width = 50;
	double orderOfConvergence;
	createConvergenceFile(csvFileName);
	std::cout << std::setw(width) << std::left << "Mesh type" << "Order of convergence" << std::endl;

	SquareCavityHeatTransfer problem(unstructuredQuadrangleDirectory + unstructuredFile);

	problem.rho = 1;
	problem.cp = 1;
	problem.k = 1;
	problem.timeImplicitCoefficient = 1.0;
	problem.timeInterval = 0.01;
	problem.insertDirichletBoundaryCondition("bottom boundary",0.0);
	problem.insertDirichletBoundaryCondition("east boundary",0.0);
	problem.insertDirichletBoundaryCondition("west boundary",0.0);
	problem.insertDirichletBoundaryCondition("top boundary",
		[](Eigen::Vector3d centroid) -> double
		{ return std::sin(M_PI*centroid.x()) * std::sinh(M_PI*centroid.y()) / std::sinh(M_PI); });


	// Element center export
	// const std::string outputDirectory = unstructuredQuadrangleDirectory + std::string("output/");
	// std::string elementCenterResult = outputDirectory + "element_center.cgns";
	std::string elementCenterResult = unstructuredQuadrangleDirectory + unstructuredFile;
	std::cout << elementCenterResult << std::endl;
	CgnsWriter cgnsElementCenterWriter(elementCenterResult, "CellCenter");


	double finalTime = 100*problem.timeInterval;
	double currentTime = problem.timeInterval;
	problem.prepareMatrix();
	do
	{
		problem.temperature = problem.nextTimeStep();
		problem.oldTemperature = problem.temperature;
		cgnsElementCenterWriter.writeTransientSolution(currentTime);

		std::vector<double> numericalSolution;
		numericalSolution.resize(problem.temperature.size());
		Eigen::VectorXd::Map(&numericalSolution[0], problem.temperature.size()) = problem.temperature;
		cgnsElementCenterWriter.writeTransientField(numericalSolution, "Temperature");

		std::cout << "Current Time: " << currentTime << std::endl;
		currentTime += problem.timeInterval;
	} while(currentTime <= finalTime);







	// // Steady numerical solution
	// unsigned iteration = 0;
	// double convergenceError;
	// constexpr double tolerance = 1.0e-8;
	// problem.prepareMatrix();
	// do
	// {
	// 	problem.oldTemperature = problem.temperature;
	// 	problem.temperature = problem.nextTimeStep();
	// 	convergenceError = (problem.temperature - problem.oldTemperature).lpNorm<Eigen::Infinity>();
	// 	++iteration;
	// } while(convergenceError > tolerance);
	// Eigen::VectorXd numericalSolution = problem.temperature;

	// // Analytical
	// Eigen::VectorXd analyticalSolution = problem.computeAnalyticalSolution();

	// // Compare
	// Eigen::VectorXd difference = numericalSolution - analyticalSolution;
	// double error = 0.0;
	// for(auto element: problem.grid2D.elements)
	// {
	// 	const unsigned index = element->getIndex();
	// 	error += element->getVolume() * difference[index] * difference[index];
	// }
	// error = std::sqrt(error);
	// // double error = std::sqrt(difference.squaredNorm()/problem.grid2D.elements.size());
	// std::cout << std::endl << "\t" << "erro: " << error << "  characteristic length:" << problem.grid2D.getCharacteristicLength() << std::endl;








	return 0;
}