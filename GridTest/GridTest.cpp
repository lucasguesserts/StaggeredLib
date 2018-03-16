#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <array>
#include <vector>
#include <string>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>

#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>

#include <Grid/GridData.hpp>
#include <Grid/Grid.hpp>
#include <Grid/Grid2D.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>

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

TestCase("grid reader from CGNS", "[GridData][CGNS]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
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
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
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
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
	section("Vertex 3 neighborhood")
	{
		const unsigned vertexIndex = 3;
		std::vector<Element*>& vertexNeighborhood = grid.verticesNeighborElements[vertexIndex];
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

TestCase("grid 2D with staggered elements", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData); // TODO: Algorith to create this grid using a GridData structure.
	section("grid vertices")
	{
		constexpr unsigned numberOfVertices = 4;
		std::vector<Vertex> vertices;
		vertices.push_back(Vertex(0.0, 0.0, 0.0, 0));
		vertices.push_back(Vertex(2.0, 0.0, 0.0, 1));
		vertices.push_back(Vertex(0.0, 2.0, 0.0, 2));
		vertices.push_back(Vertex(2.0, 2.0, 0.0, 3));
		for(Vertex& vertex: grid.vertices)
			require(vertex==vertices[vertex.getIndex()]);
	}
	section("grid triangles")
	{
		constexpr unsigned numberOfTriangles = 2;
		// 0
		require(grid.elements[0]->vertices[0]==&(grid.vertices[0]));
		require(grid.elements[0]->vertices[1]==&(grid.vertices[1]));
		require(grid.elements[0]->vertices[2]==&(grid.vertices[3]));
		// 1
		require(grid.elements[1]->vertices[0]==&(grid.vertices[0]));
		require(grid.elements[1]->vertices[1]==&(grid.vertices[3]));
		require(grid.elements[1]->vertices[2]==&(grid.vertices[2]));
	}
	section("staggered quadrangles")
	{
		constexpr unsigned index = 0;
		StaggeredQuadrangle staggeredQuadrangle(index,grid.vertices[0],grid.elements[0],grid.vertices[3],grid.elements[1]);
		grid.staggeredQuadrangles.push_back(StaggeredQuadrangle(index,grid.vertices[0],grid.elements[0],grid.vertices[3],grid.elements[1])); //TODO: remove this line and, if necessary, update the test
		section("vertices")
		{
			check(grid.staggeredQuadrangles[0].vertices[0]==&grid.vertices[0]);
			check(grid.staggeredQuadrangles[0].vertices[1]==&grid.vertices[3]);
			check(grid.staggeredQuadrangles[0].elements[0]==grid.elements[0]);
			check(grid.staggeredQuadrangles[0].elements[1]==grid.elements[1]);
		}
	}
	section("staggered triangles")
	{
		std::vector<StaggeredTriangle> staggeredTriangles = {
			{1, grid.vertices[1], grid.elements[0], grid.vertices[0]},
			{2, grid.vertices[3], grid.elements[0], grid.vertices[1]},
			{3, grid.vertices[0], grid.elements[1], grid.vertices[2]},
			{4, grid.vertices[2], grid.elements[1], grid.vertices[3]}
		};
		for(StaggeredTriangle& staggeredTriangle: staggeredTriangles)
			grid.staggeredTriangles.push_back(staggeredTriangle); //TODO: remove this line and, if necessary, update the test
		for(unsigned index=0 ; index<4 ; ++index)
		{
			check(grid.staggeredTriangles[index].vertices[0]==staggeredTriangles[index].vertices[0]);
			check(grid.staggeredTriangles[index].element==staggeredTriangles[index].element);
			check(grid.staggeredTriangles[index].vertices[1]==staggeredTriangles[index].vertices[1]);
		}
	}
}

TestCase("Compute ScalarStencil in grid, vertex by vertex", "[Grid2DInverseDistanceStencil]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DInverseDistanceStencil grid(gridData);
	auto scalarStencilTest = [&grid](const unsigned vertexIndex, ScalarStencil& correctScalarStencil) -> void
		{
			ScalarStencil scalarStencilToTest = grid.computeScalarStencil(grid.vertices[vertexIndex]);
			for(Element* element: grid.verticesNeighborElements[vertexIndex])
				check(scalarStencilToTest[element->getIndex()]==Approx(correctScalarStencil[element->getIndex()]));
		};
	section("Vertex 0")
	{
		constexpr unsigned vertexIndex = 0;
		ScalarStencil correctScalarStencil = { {0,1.0} };
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 1")
	{
		constexpr unsigned vertexIndex = 1;
		ScalarStencil correctScalarStencil = { {0,0.5}, {1,0.5} };
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 2")
	{
		constexpr unsigned vertexIndex = 2;
		ScalarStencil correctScalarStencil = { {1,1.0} };
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 3")
	{
		constexpr unsigned vertexIndex = 3;
		ScalarStencil correctScalarStencil = {
			{0,0.3451400998500395},
			{2,0.327429500749802},
			{3,0.327429500749802}
		};
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 4")
	{
		constexpr unsigned vertexIndex = 4;
		ScalarStencil correctScalarStencil = {
			{0,0.185275538023003},
			{1,0.185275538023003},
			{2,0.277913307034504},
			{4,0.175767808459745},
			{5,0.175767808459745}
		};
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 5")
	{
		constexpr unsigned vertexIndex = 5;
		ScalarStencil correctScalarStencil = { {1,0.4}, {4,0.6} };
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 6")
	{
		constexpr unsigned vertexIndex = 6;
		ScalarStencil correctScalarStencil = { {3,1.0} };
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 7")
	{
		constexpr unsigned vertexIndex = 7;
		ScalarStencil correctScalarStencil = {
			{2,0.279240779943874},
			{3,0.279240779943874},
			{5,0.441518440112253}
		};
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
	section("Vertex 8")
	{
		constexpr unsigned vertexIndex = 8;
		ScalarStencil correctScalarStencil = {
			{4,0.5},
			{5,0.5}
		};
		scalarStencilTest(vertexIndex, correctScalarStencil);
	}
}

TestCase("Compute ScalarStencil for all vertices in grid", "[Grid2DInverseDistanceStencil]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DInverseDistanceStencil grid(gridData);
	std::vector<ScalarStencil> correctScalarStencilOnVertices = {
		{ {0,1.0} },
		{ {0,0.5}, {1,0.5} },
		{ {1,1.0} },
		{ {0,0.3451400998500395}, {2,0.327429500749802}, {3,0.327429500749802} },
		{ {0,0.185275538023003}, {1,0.185275538023003}, {2,0.277913307034504}, {4,0.175767808459745}, {5,0.175767808459745} },
		{ {1,0.4}, {4,0.6} },
		{ {3,1.0} },
		{ {2,0.279240779943874}, {3,0.279240779943874}, {5,0.441518440112253} },
		{ {4,0.5}, {5,0.5} }
	};
	std::vector<ScalarStencil> toTestScalarStencilOnVertices = grid.computeScalarStencilOnVertices();
	for(Vertex& vertex: grid.vertices)
		for(Element* element: grid.verticesNeighborElements[vertex.getIndex()])
			check(toTestScalarStencilOnVertices[vertex.getIndex()][element->getIndex()]==Approx(correctScalarStencilOnVertices[vertex.getIndex()][element->getIndex()]));
}

TestCase("Compute scalar stencil for one element", "[Grid2DInverseDistanceStencil]")
{
	constexpr unsigned index = 14;
	constexpr double elementWeight = 1.0;
	Quadrangle element;
	element.setIndex(index);
	ScalarStencil computedScalarStencil = Grid2DInverseDistanceStencil::computeScalarStencilOnElement(&element);
	ScalarStencil scalarStencil = { {index,elementWeight} };
	check(computedScalarStencil==scalarStencil);
}

TestCase("ScalarStencilComputer compute area vector","[ScalarStencilComputer]")
{
	Eigen::Vector3d point[] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0} };
	Eigen::Vector3d correctAreaVector = {1.0, 1.0, 0.0};
	Eigen::Vector3d areaVector = Grid2DInverseDistanceStencil::computeAreaVector(point[0],point[1]);
	check(areaVector==correctAreaVector);
}

TestCase("ScalarStencilComputer compute average scalar stencil","[ScalarStencilComputer]")
{
	ScalarStencil scalarStencil[] = { {{0, 2.2},{4, 5.4}} , {{1, 4.8}, {4, 3.2}} };
	ScalarStencil correctScalarStencil = {{0, 1.1}, {1, 2.4}, {4, 4.3}};
	ScalarStencil averageScalarStencil = Grid2DInverseDistanceStencil::computeAverageScalarStencil(scalarStencil[0],scalarStencil[1]);
	for(auto keyValue: averageScalarStencil)
		check(averageScalarStencil[keyValue.first]==Approx(correctScalarStencil[keyValue.first]));
}

TestCase("ScalarStencilComputer compute vector stencil","[ScalarStencilComputer]")
{
	Eigen::Vector3d point[] = { {0.0, 1.0, 0.0}, {-2.0, 0.0, 0.0} };
	ScalarStencil scalarStencil[] = { {{0, 2.2},{4, 5.4}} , {{1, 4.8}, {4, 3.2}} };
	VectorStencil vectorStencil = Grid2DInverseDistanceStencil::computeVectorStencil(point[0], point[1], scalarStencil[0], scalarStencil[1]);
	Eigen::Vector3d areaVector = Grid2DInverseDistanceStencil::computeAreaVector(point[0],point[1]);
	VectorStencil correctVectorStencil = 0.5 * (scalarStencil[0]+scalarStencil[1]) * areaVector;
	check(vectorStencil==correctVectorStencil);
}

TestCase("ScalarStencilComputer compute gradient using StaggeredQuadrangle","[ScalarStencilComputer][StaggeredQuadrangle]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DInverseDistanceStencil grid(gridData);
	// create staggered elements
	const unsigned staggeredQuadrangleIndex = 0;
	StaggeredQuadrangle staggeredQuadrangle(staggeredQuadrangleIndex,grid.vertices[0],grid.elements[0],grid.vertices[3],grid.elements[1]);
	const std::array<unsigned,4> staggeredTrianglesIndices = {{1, 2, 3, 4}};
	// compute vector stencil
	std::vector<ScalarStencil> scalarStencilOnVertices = grid.computeScalarStencilOnVertices();
	VectorStencil correctVectorStencil = { { 0, {0.75,-0.75,0.0} }, { 1, {-0.75,0.75,0.0} } };
	VectorStencil vectorStencilOnStaggeredQuadrangle = grid.computeVectorStencilOnQuadrangle(staggeredQuadrangle, scalarStencilOnVertices);
	for(auto& keyValue: vectorStencilOnStaggeredQuadrangle)
	{
		for(unsigned vectorEntry=0 ; vectorEntry<3 ; ++vectorEntry)
			check(keyValue.second[vectorEntry]==Approx(correctVectorStencil[keyValue.first][vectorEntry]));
	}
}
