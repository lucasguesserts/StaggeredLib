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
#include <CgnsInterface/CgnsWriter.hpp>
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
const std::string csvFileName = gridDirectory + "facetCenter_convergence.csv";

auto analiseConvergence = [](const std::string& directory, const std::vector<std::string>& fileNames) -> RateOfConvergence
{
	std::vector<double> numericalError, characteristicLength;
	for(auto& fileName: fileNames)
	{
		FacetCenterHeatTransfer problem(directory + fileName);

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
		double error = 0.0;
		for(auto element: problem.grid2D.elements)
		{
			const unsigned index = element->getIndex();
			error += element->getVolume() * difference[index] * difference[index];
		}
		error = std::sqrt(error);
		// double error = difference.lpNorm<Eigen::Infinity>();
		numericalError.push_back(error);
		characteristicLength.push_back(problem.grid2D.getStaggeredCharacteristicLength());
		std::cout << std::endl << "\t" << "erro: " << error << "  characteristic length:" << characteristicLength.back();

		// Export
		std::string resultFileName = directory + "result_" + fileName;
		Grid2DWithStaggeredElementsExport::cgns(resultFileName, problem.grid2D);
		CgnsWriter cgnsWriter(resultFileName, "CellCenter");
		cgnsWriter.writePermanentSolution("steadySolution");
		cgnsWriter.writePermanentField("Numerical_Temperature", std::vector<double>(&numericalSolution[0], &numericalSolution[0] + numericalSolution.size()));
		cgnsWriter.writePermanentField("Analytical_Temperature", std::vector<double>(&analyticalSolution[0], &analyticalSolution[0] + analyticalSolution.size()));
		cgnsWriter.writePermanentField("error", std::vector<double>(&difference[0], &difference[0] + difference.size()));
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