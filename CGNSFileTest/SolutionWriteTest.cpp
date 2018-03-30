#include <Utils/Test.hpp>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>

#include <CGNSFile/CGNSFile.hpp>

TestCase("CGNS file structure - steady solution","[CGNSFile]")
{
	// create temporary file
	const std::string fileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	boost::filesystem::path filePath(fileName);
	const std::string tempFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS_temp.cgns";
	boost::filesystem::path tempFilePath(tempFileName);
	if(boost::filesystem::exists(tempFilePath))
		boost::filesystem::remove(tempFilePath);
	boost::filesystem::copy(filePath, tempFilePath);
	require(boost::filesystem::exists(tempFilePath));

	CGNSFile cgnsFile(tempFileName);
	constexpr unsigned numberOfElements = 6;
	const std::string solutionName = "steady solution";
	const std::string scalarFieldName = "Temperature";
	section("write and read")
	{
		Eigen::VectorXd steadySolution;
		steadySolution.resize(numberOfElements);
		steadySolution << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
		cgnsFile.writeSteadyScalarField(solutionName,scalarFieldName,steadySolution);
		Eigen::VectorXd readSolution = cgnsFile.readSteadyScalarField(solutionName,scalarFieldName);
		check(readSolution==steadySolution);
	}
	// TODO: add exceptions and create failures tests
	boost::filesystem::remove(tempFilePath);
	checkFalse(boost::filesystem::exists(tempFilePath));
}

TestCase("CGNS file structure - transient solution","[CGNSFile]")
{
	// create temporary file
	const std::string fileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	boost::filesystem::path filePath(fileName);
	const std::string tempFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS_transient.cgns";
	boost::filesystem::path tempFilePath(tempFileName);
	if(boost::filesystem::exists(tempFilePath))
		boost::filesystem::remove(tempFilePath);
	boost::filesystem::copy(filePath, tempFilePath);
	require(boost::filesystem::exists(tempFilePath));

	CGNSFile cgnsFile(tempFileName);
	constexpr double deltaT = 2;
	constexpr unsigned numberOfTimeSteps = 30;
	constexpr unsigned numberOfElements = 6;
	const std::string scalarFieldName = "Temperature";
	// write transient field
	Eigen::VectorXd transientSolution;
	transientSolution.resize(numberOfElements);
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
	{
		transientSolution << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
		transientSolution *= timeStep;
		cgnsFile.writeTransientScalarField(scalarFieldName,timeStep,transientSolution);
	}
	Eigen::VectorXd timeInstants;
	timeInstants.resize(numberOfTimeSteps);
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		timeInstants[timeStep] = timeStep*deltaT;
	cgnsFile.writeTransientInformation(scalarFieldName,timeInstants);
	section("read transient solution")
	{
		for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		{
			Eigen::VectorXd readTransientScalarField = cgnsFile.readTransientScalarField(scalarFieldName,timeStep);
			transientSolution << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
			transientSolution *= timeStep;
			check(readTransientScalarField==transientSolution);
		}
	}
	section("read number of time steps")
	{
		const unsigned readNumberOfTimeSteps = cgnsFile.readNumberOfTimeSteps();
		check(readNumberOfTimeSteps==numberOfTimeSteps);
	}
	section("read time steps")
	{
		Eigen::VectorXd allReadTimeInstants = cgnsFile.readAllTimeInstants();
		check(allReadTimeInstants==timeInstants);
	}
	// TODO: add exceptions and create failures tests
	boost::filesystem::remove(tempFilePath);
	checkFalse(boost::filesystem::exists(tempFilePath));
}