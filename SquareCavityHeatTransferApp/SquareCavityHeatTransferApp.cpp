#include <array>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <iostream>
#include <cmath>

#include <CGNSFile/CGNSFile.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/Boundary.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

int main()
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "heatDiffusion_25_25.cgns";
	const std::string cgnsResultFileName = CGNSFile::gridDirectory + "result_heatDiffusion_25_25.cgns";
	boost::filesystem::path filePath(cgnsGridFileName);
	boost::filesystem::path resultFilePath(cgnsResultFileName);
	if(boost::filesystem::exists(resultFilePath))
		boost::filesystem::remove(resultFilePath);
	if(!boost::filesystem::exists(filePath))
		throw std::runtime_error(std::string("File '") + cgnsGridFileName + std::string("' does not exist!"));
	boost::filesystem::copy(filePath, resultFilePath);

	CGNSFile cgnsFile(cgnsResultFileName);
	GridData gridData(cgnsFile);
	SquareCavityHeatTransfer problem(gridData);
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
		dirichlet.prescribedValue = 0.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "east boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue = 0.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "top boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue = 1.0;
		problem.dirichletBoundaries.push_back(dirichlet);
		boundaryName = "west boundary";
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue = 0.0;
		problem.dirichletBoundaries.push_back(dirichlet);

	double tolerance = 1e-3;
	unsigned timeStep = 0;
	double error;
	const std::string scalarFieldName = "Temperature";
	do
	{
		std::cout << "\ttime step: " << timeStep << std::endl;
		problem.oldTemperature = problem.temperature;
		cgnsFile.writeTransientScalarField(scalarFieldName,timeStep,problem.oldTemperature);
		problem.temperature = problem.nextTimeStep();
		error = (problem.temperature - problem.oldTemperature).norm();
		++timeStep;
		std::cout << "\t\terror = " << error << std::endl;
	} while(error > tolerance);
	const unsigned numberOfTimeSteps = timeStep;
	Eigen::VectorXd timeInstants;
	timeInstants.resize(numberOfTimeSteps);
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		timeInstants[timeStep] = timeStep * problem.timeInterval;
	cgnsFile.writeTransientInformation(scalarFieldName,timeInstants);

	return 0;
}