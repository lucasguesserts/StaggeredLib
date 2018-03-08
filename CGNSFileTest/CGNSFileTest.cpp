#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <array>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>

TestCase("Element definition", "[ElementDefinition]")
{
	constexpr unsigned numberOfVertices = 4;
	constexpr unsigned index = 7;
	Eigen::Matrix<unsigned,numberOfVertices,1> connectivity;
	connectivity << 1, 7, 4, 2;
	ElementDefinition<numberOfVertices> elementDefinition;
	elementDefinition.index = index;
	elementDefinition.connectivity << 1, 7, 4, 2;
	check(elementDefinition.index==index);
	check(elementDefinition.connectivity.size()==numberOfVertices);
	check(elementDefinition.connectivity==connectivity);
}

TestCase("CGNS file structure basic - coordinates and element connectivity", "[CGNSFile]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	section("read coordinates X")
	{
		std::vector<double> verticesCoordinateX{0.0, 1.5, 3.0, 0.0, 1.5, 3.0, 0.0, 1.5, 3.0};
		std::vector<double> readCoordinateX = cgnsFile.readCoordinate("CoordinateX");
		check(readCoordinateX==verticesCoordinateX);
	}
	section("read coordinates Y")
	{
		std::vector<double> verticesCoordinateY{0.0, 0.0, 0.0, 1.5, 1.5, 1.5, 3.0, 3.0, 3.0};
		std::vector<double> readCoordinateY = cgnsFile.readCoordinate("CoordinateY");
		check(readCoordinateY==verticesCoordinateY);
	}
	section("read quadrangles")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		std::vector< ElementDefinition<4> > quadrangle(numberOfQuadrangles);
			quadrangle[0].index = 0;
			quadrangle[0].connectivity << 0, 1, 4, 3;
			quadrangle[1].index = 1;
			quadrangle[1].connectivity << 1, 2, 5, 4;
		std::vector< ElementDefinition<4> > readQuadrangle = cgnsFile.readElementsDefinition<4,cgns::QUAD_4>();
		check(readQuadrangle==quadrangle);
	}
	section("read triangles")
	{
		constexpr unsigned numberOfTriangles = 4;
		std::vector< ElementDefinition<3> > triangle(numberOfTriangles);
			triangle[0].index = 2;
			triangle[0].connectivity << 3, 4, 7;
			triangle[1].index = 3;
			triangle[1].connectivity << 3, 7, 6;
			triangle[2].index = 4;
			triangle[2].connectivity << 4, 5, 8;
			triangle[3].index = 5;
			triangle[3].connectivity << 4, 8, 7;
		std::vector< ElementDefinition<3> > readTriangle = cgnsFile.readElementsDefinition<3,cgns::TRI_3>();
		check(readTriangle==triangle);
	}
	section("read lines")
	{
		constexpr unsigned numberOfLines = 8;
		std::vector< ElementDefinition<2> > line(numberOfLines);
			line[0].index = 6;
			line[0].connectivity << 1, 0;
			line[1].index = 7;
			line[1].connectivity << 2, 1;
			line[2].index = 8;
			line[2].connectivity << 6, 7;
			line[3].index = 9;
			line[3].connectivity << 7, 8;
			line[4].index = 10;
			line[4].connectivity << 3, 0;
			line[5].index = 11;
			line[5].connectivity << 6, 3;
			line[6].index = 12;
			line[6].connectivity << 5, 2;
			line[7].index = 13;
			line[7].connectivity << 8, 5;
		std::vector< ElementDefinition<2> > readLine = cgnsFile.readElementsDefinition<2,cgns::BAR_2>();
		check(readLine==line);
	}
}

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
