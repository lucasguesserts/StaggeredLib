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

int main(int argc, char* argv[])
{
	if(argc!=2)
	{
		std::cout << "Usage: cgns_file_name.cgns" << std::endl;
		std::cout << "output: result_cgns_file_name.cgns" << std::endl;
		return 1;
	}
	const std::string fileName = argv[1];

	FacetCenterHeatTransfer problem(fileName);
	problem.insertDirichletBoundaryCondition("bottom boundary",0.0);
	problem.insertDirichletBoundaryCondition("east boundary",0.0);
	problem.insertDirichletBoundaryCondition("west boundary",0.0);
	problem.insertDirichletBoundaryCondition("top boundary",1.0);

	// Steady numerical solution
	problem.addDiffusiveTerm();
	problem.applyBoundaryConditions();
	problem.linearSystem.computeLU();
	Eigen::VectorXd numericalSolution = problem.linearSystem.solve();

	// Export
	std::string resultFileName = "./result.cgns";
	Grid2DWithStaggeredElementsExport::cgns(resultFileName, problem.grid2D);
	CgnsWriter cgnsWriter(resultFileName, "CellCenter");
	cgnsWriter.writePermanentSolution("steadySolution");
	cgnsWriter.writePermanentField("temperatura", std::vector<double>(&numericalSolution[0], &numericalSolution[0] + numericalSolution.size()));
	std::cout << "Successfully write solution to '" << resultFileName << "'." << std::endl;

	return 0;
}