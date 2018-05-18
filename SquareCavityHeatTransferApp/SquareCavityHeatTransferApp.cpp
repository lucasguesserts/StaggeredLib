#include <array>
#include <vector>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <iostream>
#include <cmath>

#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/Boundary.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>
#include <CgnsInterface/CgnsWriter.hpp>

int main()
{
	const std::string cgnsGridFileName = gridDirectory + "heatDiffusion_25_25.cgns";
	const std::string cgnsResultFileName = gridDirectory + "result_heatDiffusion_25_25.cgns";
	boost::filesystem::copy_file(cgnsGridFileName, cgnsResultFileName, boost::filesystem::copy_option::overwrite_if_exists);

	SquareCavityHeatTransfer problem(cgnsResultFileName);
	problem.rho = 1;
	problem.cp = 1;
	problem.k = 1;
	problem.timeImplicitCoefficient = 1.0;
	problem.timeInterval = 1.0/pow(2,8);
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
		for(auto& entry: dirichlet.prescribedValue)
			entry = 1.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "west boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = 0.0;
		problem.dirichletBoundaries.push_back(dirichlet);

	double tolerance = 1e-3;
	unsigned timeStep = 0;
	double error;
	const std::string scalarFieldName = "Temperature";

	CgnsWriter cgnsWriter(cgnsResultFileName, "CellCenter");



	do
	{
		std::cout << "\ttime step: " << timeStep << std::endl;
		problem.oldTemperature = problem.temperature;

		cgnsWriter.writeTimeStep(timeStep * problem.timeInterval);
		cgnsWriter.writeTransientField(
			std::vector<double>(problem.oldTemperature.data(), problem.oldTemperature.data()+problem.oldTemperature.size()),
			scalarFieldName);
		problem.temperature = problem.nextTimeStep();
		error = (problem.temperature - problem.oldTemperature).norm();
		++timeStep;
		std::cout << "\t\terror = " << error << std::endl;
	} while(error > tolerance);

	return 0;
}