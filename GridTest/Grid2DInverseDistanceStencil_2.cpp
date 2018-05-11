#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <vector>
#include <string>

#include <CGNSFile/CGNSFile.hpp>

#include <GeometricEntity/Test.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>

#include <Stencil/Test.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>

#include <Grid/Grid2DInverseDistanceStencil.hpp>

TestCase("Compute ScalarStencil in grid, vertex by vertex", "[Grid2DInverseDistanceStencil]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DInverseDistanceStencil grid(gridData);
	auto scalarStencilTest = [&grid](const unsigned vertexIndex, ScalarStencil& correctScalarStencil) -> void
		{
			ScalarStencil scalarStencilToTest = grid.computeScalarStencil(grid.vertices[vertexIndex]);
			check(scalarStencilToTest==correctScalarStencil);
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
	check(toTestScalarStencilOnVertices==correctScalarStencilOnVertices);
}

TestCase("Compute ScalarStencil based on staggered elements for all vertices", "[Grid2DInverseDistanceStencil]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DInverseDistanceStencil grid(gridData);
	std::vector<ScalarStencil> correctScalarStencilOnVertices = {
		{ {0,0.36939806252}, {2,0.26120387496}, {4,0.36939806252} },
		{ {0,0.5},           {1,0.5} },
		{ {3,0.5},           {4,0.5} },
		{ {1,0.36939806252}, {2,0.26120387496}, {3,0.36939806252} }
	};
	std::vector<ScalarStencil> scalarStencilOnVertices = grid.computeScalarStencilOnVerticesUsingStaggeredElements();
	check(scalarStencilOnVertices==correctScalarStencilOnVertices);
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

TestCase("Grid2DInverseDistanceStencil compute area vector","[Grid2DInverseDistanceStencil]")
{
	// TODO: move this function to somewhere in the StaggeredElements
	Eigen::Vector3d point[] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0} };
	Eigen::Vector3d correctAreaVector = {1.0, 1.0, 0.0};
	Eigen::Vector3d areaVector = Grid2DInverseDistanceStencil::computeAreaVector(point[0],point[1]);
	check(areaVector==correctAreaVector);
}

TestCase("Grid2DInverseDistanceStencil compute average scalar stencil","[Grid2DInverseDistanceStencil]")
{
	ScalarStencil scalarStencil[] = { {{0, 2.2},{4, 5.4}} , {{1, 4.8}, {4, 3.2}} };
	ScalarStencil correctScalarStencil = {{0, 1.1}, {1, 2.4}, {4, 4.3}};
	ScalarStencil averageScalarStencil = Grid2DInverseDistanceStencil::computeAverageScalarStencil(scalarStencil[0],scalarStencil[1]);
	check(averageScalarStencil==correctScalarStencil);
}

TestCase("Grid2DInverseDistanceStencil compute vector stencil","[Grid2DInverseDistanceStencil]")
{
	Eigen::Vector3d point[] = { {0.0, 1.0, 0.0}, {-2.0, 0.0, 0.0} };
	ScalarStencil scalarStencil[] = { {{0, 2.2},{4, 5.4}} , {{1, 4.8}, {4, 3.2}} };
	VectorStencil vectorStencil = Grid2DInverseDistanceStencil::computeVectorStencil(point[0], point[1], scalarStencil[0], scalarStencil[1]);
	Eigen::Vector3d areaVector = Grid2DInverseDistanceStencil::computeAreaVector(point[0],point[1]);
	VectorStencil correctVectorStencil = 0.5 * (scalarStencil[0]+scalarStencil[1]) * areaVector;
	check(vectorStencil==correctVectorStencil);
}

TestCase("Grid2DInverseDistanceStencil compute gradient using StaggeredQuadrangle","[Grid2DInverseDistanceStencil][StaggeredQuadrangle]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DInverseDistanceStencil grid(gridData);
	std::vector<ScalarStencil> scalarStencilOnVertices = grid.computeScalarStencilOnVertices();
	VectorStencil correctVectorStencil = { { 0, {0.75,-0.75,0.0} }, { 1, {-0.75,0.75,0.0} } };
	VectorStencil vectorStencilOnStaggeredQuadrangle = grid.computeVectorStencilOnQuadrangle(*grid.staggeredQuadrangles[0], scalarStencilOnVertices);
	check(vectorStencilOnStaggeredQuadrangle==correctVectorStencil);
}