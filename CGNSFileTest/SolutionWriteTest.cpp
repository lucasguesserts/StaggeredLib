#include <Utils/Test.hpp>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <CgnsInterface/CgnsReader/CgnsReader2D.hpp>
#include <CgnsInterface/CgnsWriter.hpp>

#include <CGNSFile/CGNSFile.hpp>

TestCase("CGNS file structure - steady solution","[CGNSFile]")
{
	const std::string fileName = gridDirectory + "GridReaderTest_CGNS.cgns";
	const std::string tempFileName = gridDirectory + "GridReaderTest_CGNS_temp.cgns";
	boost::filesystem::copy_file(fileName, tempFileName, boost::filesystem::copy_option::overwrite_if_exists);
	const std::string solutionName = "steady solution";
	const std::string scalarFieldName = "Temperature";
	std::vector<double> steadySolution = {0.0, 1.0, 3.0, 2.0, 5.0, 4.0};
	std::vector<double> readSolution;
	{
		std::unique_ptr<CgnsWriter> cgnsWriter = std::make_unique<CgnsWriter>(tempFileName, "CellCenter");
		cgnsWriter->writePermanentField(solutionName, scalarFieldName, steadySolution);
	}
	{
		std::unique_ptr<CgnsReader2D> cgnsReader = std::make_unique<CgnsReader2D>(tempFileName);
		readSolution = cgnsReader->readField(solutionName, scalarFieldName);
	}
	check(readSolution==steadySolution);
	boost::filesystem::remove_all(tempFileName);
}

TestCase("CGNS file structure - transient solution","[CGNSFile]")
{
	const std::string fileName = gridDirectory + "GridReaderTest_CGNS.cgns";
	const std::string tempFileName = gridDirectory + "GridReaderTest_CGNS_transient.cgns";
	boost::filesystem::copy_file(fileName, tempFileName, boost::filesystem::copy_option::overwrite_if_exists);

	constexpr double deltaT = 2;
	constexpr unsigned numberOfTimeSteps = 30;
	constexpr unsigned numberOfElements = 6;
	const std::string scalarFieldName = "Temperature";

	std::vector<double> timeInstants(numberOfTimeSteps, 0.0);
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		timeInstants[timeStep] = timeStep*deltaT;

	// Create transient field
	std::vector<std::vector<double>> transientSolution(30, {0.0, 1.0, 3.0, 2.0, 5.0, 4.0});
	{
		std::unique_ptr<CgnsWriter> cgnsWriter = std::make_unique<CgnsWriter>(tempFileName, "CellCenter");
		for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		{
			std::transform(transientSolution[timeStep].begin(), transientSolution[timeStep].end(), transientSolution[timeStep].begin(), [&timeStep](const double& x){return timeStep*x;});
			cgnsWriter->writeTimeStep(timeInstants[timeStep]);
			cgnsWriter->writeTransientField(transientSolution[timeStep], scalarFieldName);
		}
	}

	section("read transient solution")
	{
		CgnsReader2D cgnsReader(tempFileName);
		for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		{
			std::vector<double> readTransientScalarField = cgnsReader.readField(static_cast<int>(timeStep+1), scalarFieldName);
			check(readTransientScalarField==transientSolution[timeStep]);
		}
	}
	section("read number of time steps")
	{
		CgnsReader2D cgnsReader(tempFileName);
		const unsigned readNumberOfTimeSteps = static_cast<unsigned>( cgnsReader.readNumberOfTimeSteps() );
		check(readNumberOfTimeSteps==numberOfTimeSteps);
	}
	section("read time steps")
	{
		CgnsReader2D cgnsReader(tempFileName);
		std::vector<double> allReadTimeInstants = cgnsReader.readTimeInstants();
		check(allReadTimeInstants==timeInstants);
	}
	boost::filesystem::remove_all(tempFileName);
}