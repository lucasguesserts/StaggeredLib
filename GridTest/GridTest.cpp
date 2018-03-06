#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <array>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>

#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <Grid/ElementDefinition.hpp>
#include <Grid/CGNSFile.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid.hpp>
#include <Grid/Grid2D.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>

TestCase("Element definition for GridData", "[ElementDefinition]")
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

TestCase("Grid data structure", "[Grid][GridData][ElementDefinition]")
{
	const unsigned numberOfVertices = 7;
	GridData gridData;
	gridData.coordinates.resize(numberOfVertices,Eigen::NoChange);
	gridData.coordinates <<
		0.0, 0.0, 0.0,
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		1.0, 1.0, 0.0,
		0.5, 2.0, 0.0,
		2.0, 1.5, 0.0,
		1.5, 2.5, 0.0;
	section("dimension")
	{
		const unsigned dimension = 2;
		gridData.dimension = dimension;
		check(gridData.dimension==dimension);
	}
	section("quadrangle connectivity")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		std::array<ElementDefinition<4>,numberOfQuadrangles> quadrangleDefinition;
		// Two answer quadrangles
		quadrangleDefinition[0].index = 12;
		quadrangleDefinition[0].connectivity << 0, 1, 3, 2;
		quadrangleDefinition[1].index = 17;
		quadrangleDefinition[1].connectivity << 3, 5, 6, 4;
		// Two quadrangles in GridData to test
		gridData.quadrangle.resize(numberOfQuadrangles);
		gridData.quadrangle[0].index = 12;
		gridData.quadrangle[0].connectivity << 0, 1, 3, 2;
		gridData.quadrangle[1].index = 17;
		gridData.quadrangle[1].connectivity << 3, 5, 6, 4;
		for(unsigned quadrangleCount=0 ; quadrangleCount<numberOfQuadrangles ; ++quadrangleCount)
			check(gridData.quadrangle[quadrangleCount]==quadrangleDefinition[quadrangleCount]);
	}
	section("triangle connectivity")
	{
		constexpr unsigned numberOfTriangles = 2;
		std::array<ElementDefinition<3>,numberOfTriangles> triangleDefinition;
		// Two answer triangles
		triangleDefinition[0].index = 48;
		triangleDefinition[0].connectivity << 5, 7, 1;
		triangleDefinition[1].index = 94;
		triangleDefinition[1].connectivity << 8, 1, 6;
		// Two triangles in GridData to test
		gridData.triangle.resize(numberOfTriangles);
		gridData.triangle[0].index = 48;
		gridData.triangle[0].connectivity << 5, 7, 1;
		gridData.triangle[1].index = 94;
		gridData.triangle[1].connectivity << 8, 1, 6;
		for(unsigned triangleCount=0 ; triangleCount<numberOfTriangles ; ++triangleCount)
			check(gridData.triangle[triangleCount]==triangleDefinition[triangleCount]);
	}
}

TestCase("CGNS file structure basic - coordinates and element connectivity", "[CGNSFile]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
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
	const std::string fileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	boost::filesystem::path filePath(fileName);
	const std::string tempFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS_temp.cgns";
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
	const std::string fileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	boost::filesystem::path filePath(fileName);
	const std::string tempFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS_transient.cgns";
	boost::filesystem::path tempFilePath(tempFileName);
	if(boost::filesystem::exists(tempFilePath))
		boost::filesystem::remove(tempFilePath);
	boost::filesystem::copy(filePath, tempFilePath);
	require(boost::filesystem::exists(tempFilePath));

	CGNSFile cgnsFile(tempFileName);
	constexpr double deltaT = 2;
	constexpr unsigned numberOfTimeSteps = 3;
	constexpr unsigned numberOfElements = 6;
	const std::string scalarFieldName = "Temperature";
	section("write transient solution")
	{
		Eigen::VectorXd transientSolution;
		transientSolution.resize(numberOfElements);
		for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		{
			transientSolution << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
			transientSolution *= timeStep;
			cgnsFile.writeTransientScalarField(scalarFieldName,timeStep);
		}
		Eigen::VectorXd timeInstants;
		timeInstants.resize(numberOfTimeSteps);
		timeInstants << 0*deltaT, 1*deltaT, 2*deltaT;
		cgnsFile.writeTransientInformation(scalarFieldName,timeInstants);
	}
	// TODO: add exceptions and create failures tests
	//boost::filesystem::remove(tempFilePath);
	//checkFalse(boost::filesystem::exists(tempFilePath));
}

TestCase("grid reader from CGNS", "[GridData][CGNS]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	section("dimension")
	{
		const unsigned dimension = 2;
		check(gridData.dimension==dimension);
	}
	section("coordinates")
	{
		const unsigned numberOfVertices = 9;
		Eigen::MatrixXd verticesCoordinates;
		verticesCoordinates.resize(numberOfVertices,3);
		verticesCoordinates << 0.0, 0.0, 0.0,
							   1.5, 0.0, 0.0,
							   3.0, 0.0, 0.0,
							   0.0, 1.5, 0.0,
							   1.5, 1.5, 0.0,
							   3.0, 1.5, 0.0,
							   0.0, 3.0, 0.0,
							   1.5, 3.0, 0.0,
							   3.0, 3.0, 0.0;
		check(gridData.coordinates==verticesCoordinates);
	}
	section("quadrangle connectivity")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		constexpr unsigned verticesPerQuadrangle = 4;
		std::array<ElementDefinition<4>,numberOfQuadrangles> quadrangleDefinition;
		quadrangleDefinition[0].index = 0;
		quadrangleDefinition[0].connectivity << 0, 1, 4, 3;
		quadrangleDefinition[1].index = 1;
		quadrangleDefinition[1].connectivity << 1, 2, 5, 4;
		for(unsigned count=0 ; count<numberOfQuadrangles ; ++count)
			check(gridData.quadrangle[count]==quadrangleDefinition[count]);
	}
	section("triangle connectivity")
	{
		constexpr unsigned numberOfTriangles = 4;
		constexpr unsigned verticesPerTriangle = 3;
		std::array<ElementDefinition<3>,numberOfTriangles> triangleDefinition;
		triangleDefinition[0].index = 2;
		triangleDefinition[0].connectivity << 3, 4, 7;
		triangleDefinition[1].index = 3;
		triangleDefinition[1].connectivity << 3, 7, 6;
		triangleDefinition[2].index = 4;
		triangleDefinition[2].connectivity << 4, 5, 8;
		triangleDefinition[3].index = 5;
		triangleDefinition[3].connectivity << 4, 8, 7;
		for(unsigned count=0 ; count<numberOfTriangles ; ++count)
			check(gridData.triangle[count]==triangleDefinition[count]);
	}
}

TestCase("Grid structure", "[Grid]")
{
	const unsigned dimension = 2;
	const unsigned numberOfVertices = 4;
	GridData gridData;
	gridData.dimension = dimension;
	gridData.coordinates.resize(numberOfVertices,Eigen::NoChange);
	gridData.coordinates <<
		2.0, -5.0,  3.0,
		4.0, -1.0,  6.0,
		4.0,  3.0,  3.4,
		3.0,  7.0, -2.0;
	Grid grid(gridData);
	check(grid.vertices.size()==numberOfVertices);
	check(grid.dimension==dimension);
	for(unsigned vertexIndex=0 ; vertexIndex<numberOfVertices ; ++vertexIndex)
	{
		for(unsigned i=0 ; i<3 ; ++i)
			check(grid.vertices[vertexIndex](i)==gridData.coordinates(vertexIndex,i));
		check(grid.vertices[vertexIndex].getIndex()==vertexIndex);
	}
}

TestCase("Grid 2D build", "[Grid][Grid2D]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2D grid2D(gridData);
	section("vertices")
	{
		const unsigned numberOfVertices = 9;
		std::vector<Vertex> vertices;
		vertices.push_back(Vertex(0.0, 0.0, 0.0, 0));
		vertices.push_back(Vertex(1.5, 0.0, 0.0, 1));
		vertices.push_back(Vertex(3.0, 0.0, 0.0, 2));
		vertices.push_back(Vertex(0.0, 1.5, 0.0, 3));
		vertices.push_back(Vertex(1.5, 1.5, 0.0, 4));
		vertices.push_back(Vertex(3.0, 1.5, 0.0, 5));
		vertices.push_back(Vertex(0.0, 3.0, 0.0, 6));
		vertices.push_back(Vertex(1.5, 3.0, 0.0, 7));
		vertices.push_back(Vertex(3.0, 3.0, 0.0, 8));
		for(unsigned i=0 ; i<grid2D.vertices.size() ; ++i)
			check(grid2D.vertices[i]==vertices[i]);
	}
	section("Elements index")
	{
		for(unsigned elementIndex=0 ; elementIndex<grid2D.elements.size() ; ++elementIndex)
			check(grid2D.elements[elementIndex]->getIndex()==elementIndex);
	}
	section("triangles")
	{
		constexpr unsigned numberOfTriangles = 4;
		section("triangular elements centroid")
		{
			constexpr unsigned trianglesOffset = 2;
			std::array<Eigen::Vector3d,4> triangleCentroid;
			triangleCentroid[0] << 1.0, 2.0, 0.0;
			triangleCentroid[1] << 0.5, 2.5, 0.0;
			triangleCentroid[2] << 2.5, 2.0, 0.0;
			triangleCentroid[3] << 2.0, 2.5, 0.0;
			for(unsigned elementIndex=0 ; elementIndex<numberOfTriangles ; ++elementIndex)
				check(grid2D.elements[elementIndex+trianglesOffset]->getCentroid()==triangleCentroid[elementIndex]);
		}
		section("elements vertices")
		{
			// 2
			check(grid2D.elements[2]->vertices[0]==&(grid2D.vertices[3]));
			check(grid2D.elements[2]->vertices[1]==&(grid2D.vertices[4]));
			check(grid2D.elements[2]->vertices[2]==&(grid2D.vertices[7]));
			// 3
			check(grid2D.elements[3]->vertices[0]==&(grid2D.vertices[3]));
			check(grid2D.elements[3]->vertices[1]==&(grid2D.vertices[7]));
			check(grid2D.elements[3]->vertices[2]==&(grid2D.vertices[6]));
			// 4
			check(grid2D.elements[4]->vertices[0]==&(grid2D.vertices[4]));
			check(grid2D.elements[4]->vertices[1]==&(grid2D.vertices[5]));
			check(grid2D.elements[4]->vertices[2]==&(grid2D.vertices[8]));
			// 5
			check(grid2D.elements[5]->vertices[0]==&(grid2D.vertices[4]));
			check(grid2D.elements[5]->vertices[1]==&(grid2D.vertices[8]));
			check(grid2D.elements[5]->vertices[2]==&(grid2D.vertices[7]));
		}
	}
	section("quadrangles")
	{
		constexpr unsigned numberOfQuadrangles = 2;
		section("quadrangular elements centroid")
		{
			Eigen::Vector3d quadrangleCentroid[numberOfQuadrangles];
			quadrangleCentroid[0] << 0.75, 0.75, 0.0;
			quadrangleCentroid[1] << 2.25, 0.75, 0.0;
			for(unsigned elementIndex=0 ; elementIndex<numberOfQuadrangles ; ++elementIndex)
				check(grid2D.elements[elementIndex]->getCentroid()==quadrangleCentroid[elementIndex]);
		}
		section("elements vertices")
		{
			// 0
			check(grid2D.elements[0]->vertices[0]==&(grid2D.vertices[0]));
			check(grid2D.elements[0]->vertices[1]==&(grid2D.vertices[1]));
			check(grid2D.elements[0]->vertices[2]==&(grid2D.vertices[4]));
			check(grid2D.elements[0]->vertices[3]==&(grid2D.vertices[3]));
			// 1
			check(grid2D.elements[1]->vertices[0]==&(grid2D.vertices[1]));
			check(grid2D.elements[1]->vertices[1]==&(grid2D.vertices[2]));
			check(grid2D.elements[1]->vertices[2]==&(grid2D.vertices[5]));
			check(grid2D.elements[1]->vertices[3]==&(grid2D.vertices[4]));
		}
	}
}

TestCase("Grid that define vertex with neighbors elements", "[Grid2DVerticesWithNeighborElements]")
{
	const std::string cgnsGridFileName = GridData::projectGridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
	section("Vertex 3 neighborhood")
	{
		const unsigned vertexIndex = 3;
		std::vector<const Element*>& vertexNeighborhood = grid.verticesNeighborElements[vertexIndex];
		section("Number of neighbor elements")
		{
			const unsigned numberOfNeighborElements = 3;
			check(vertexNeighborhood.size()==numberOfNeighborElements);
		}
		section("Neighbor elements")
		{
			check(contains(vertexNeighborhood, grid.elements[0]));
			check(contains(vertexNeighborhood, grid.elements[2]));
			check(contains(vertexNeighborhood, grid.elements[3]));
		}
	}
}
