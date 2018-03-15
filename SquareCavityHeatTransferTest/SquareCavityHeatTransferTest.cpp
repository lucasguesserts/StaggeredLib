#include <array>

#include <boost/filesystem.hpp>

#include <Utils/Test.hpp>
#include <CGNSFile/CGNSFile.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/ScalarStencilComputer.hpp>
#include <Stencil/VectorStencil.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>

TestCase("Analytical solution", "[SquareCavityHeatTransfer]")
{
	const unsigned numberOfPoints = 4;
	Eigen::Matrix<double,4,3> coordinates;
	coordinates << 0.1, 0.1, 0.0,
	               0.5, 0.4, 0.0,
	               0.9, 1.0, 0.0,
	               0.7, 3.0, 0.0;
	Eigen::Vector4d solution, correctSolution;
	correctSolution << 0.008545119856,
	                   0.1397977728,
	                   0.3090169944,
	                   434.0323775;
	solution = SquareCavityHeatTransfer::computeAnalyticalSolution(coordinates);
	for(unsigned pointIndex=0 ; pointIndex<numberOfPoints ; ++pointIndex)
	{
		check(solution[pointIndex]==Approx(correctSolution[pointIndex]));
	}
}

TestCase("Linear system build", "[Eigen][EigenSolver]")
{
	const unsigned linearSystemSize = 3;
	EigenLinearSystem linearSystem;
	linearSystem.setSize(linearSystemSize);
	linearSystem.matrix << 0.462026696884809, 0.289318074606471, 0.418654423757951,
	                       0.111618554500326, 0.846885511973542, 0.805839771614926,
	                       0.664857365801401, 0.240344399557209, 0.348561451918039;
	linearSystem.independent << 0.263941635488370, 0.246987790661933, 0.706521526629329;
	Eigen::VectorXd solution;
	solution.resize(linearSystemSize);
	solution << 1.74452683050882, 3.77822561620858, -3.90581157526298;
	Eigen::VectorXd linearSystemSolution = linearSystem.solve();
	for(unsigned index=0 ; index<linearSystemSize ; ++index)
		check(solution[index]==Approx(linearSystemSolution[index]));
}

TestCase("Add accumulation term", "[SquareCavityHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	SquareCavityHeatTransfer problem(gridData);
	problem.rho = 1;
	problem.cp = 1;
	problem.oldTemperature << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
	problem.addAccumulationTerm();
	section("independent")
	{
		const unsigned numberOfElements = problem.grid2D.elements.size();
		Eigen::VectorXd independent;
		independent.resize(numberOfElements);
		independent << 2.250 * 0.0,
					   2.250 * 1.0,
		               1.125 * 3.0,
					   1.125 * 2.0,
					   1.125 * 5.0,
					   1.125 * 4.0;
		check(problem.linearSystem.independent==independent);
	}
	section("matrix")
	{
		const unsigned numberOfElements = problem.grid2D.elements.size();
		Eigen::MatrixXd matrix;
		matrix.resize(numberOfElements,numberOfElements);
		matrix << 2.250, 0.000, 0.000, 0.000, 0.000, 0.000,
		          0.000, 2.250, 0.000, 0.000, 0.000, 0.000,
		          0.000, 0.000, 1.125, 0.000, 0.000, 0.000,
		          0.000, 0.000, 0.000, 1.125, 0.000, 0.000,
		          0.000, 0.000, 0.000, 0.000, 1.125, 0.000,
		          0.000, 0.000, 0.000, 0.000, 0.000, 1.125;
		check(problem.linearSystem.matrix==matrix);
	}
	section("solve")
	{
		const unsigned numberOfElements = problem.grid2D.elements.size();
		Eigen::VectorXd temperature;
		temperature.resize(numberOfElements);
		temperature << 0.0, 1.0, 3.0, 2.0, 5.0, 4.0;
		problem.temperature = problem.linearSystem.solve();
		check(problem.temperature==temperature);
	}
}

TestCase("Add scalar stencil to eigen linear system", "[ScalarStencil][EigenLinearSystem]")
{
	const unsigned size = 3;
	std::vector<ScalarStencil> scalarStencil = {
		{{0, 0.6}, {1, 1.9}, {2, 7.6}},
		{{2, 8.9}, {0, 5.2}, {1, 8.4}},
		{{2, 3.7}, {1, 2.0}}
	};
	EigenLinearSystem linearSystem;
	linearSystem.setSize(size);
	for(unsigned line=0 ; line<size ; ++line)
		linearSystem.addScalarStencil(line,scalarStencil[line]);
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size,size);
	matrix << scalarStencil[0][0], scalarStencil[0][1], scalarStencil[0][2],
	          scalarStencil[1][0], scalarStencil[1][1], scalarStencil[1][2],
	          scalarStencil[2][0], scalarStencil[2][1], scalarStencil[2][2];
	check(linearSystem.matrix==matrix);
}

TestCase("Compute ScalarStencil in grid, vertex by vertex", "[ScalarStencilComputer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
	auto scalarStencilTest = [&grid](const unsigned vertexIndex, ScalarStencil& correctScalarStencil) -> void
		{
			ScalarStencil scalarStencilToTest = ScalarStencilComputer::inverseDistance(grid.vertices[vertexIndex], grid.verticesNeighborElements[vertexIndex]);
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

TestCase("Compute ScalarStencil for all vertices in grid", "[ScalarStencilComputer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
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
	std::vector<ScalarStencil> toTestScalarStencilOnVertices = ScalarStencilComputer::inverseDistance(grid);
	for(Vertex& vertex: grid.vertices)
		for(Element* element: grid.verticesNeighborElements[vertex.getIndex()])
			check(toTestScalarStencilOnVertices[vertex.getIndex()][element->getIndex()]==Approx(correctScalarStencilOnVertices[vertex.getIndex()][element->getIndex()]));
}

TestCase("Compute ScalarStencil for all elements in grid", "[ScalarStencilComputer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
	std::vector<ScalarStencil> correctScalarStencilOnElements = {
		{ {0,1.0} },
		{ {1,1.0} },
		{ {2,1.0} },
		{ {3,1.0} },
		{ {4,1.0} },
		{ {5,1.0} }
	};
	std::vector<ScalarStencil> toTestScalarStencilOnElements = ScalarStencilComputer::elements(grid);
	for(Element* element: grid.elements)
	{
		const unsigned elementIndex = element->getIndex();
		check(toTestScalarStencilOnElements[elementIndex][elementIndex]==Approx(correctScalarStencilOnElements[elementIndex][elementIndex]));
	}
}

TestCase("ScalarStencilComputer compute area vector","[ScalarStencilComputer]")
{
	Eigen::Vector3d point[] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0} };
	Eigen::Vector3d correctAreaVector = {1.0, 1.0, 0.0};
	Eigen::Vector3d areaVector = ScalarStencilComputer::computeAreaVector(point[0],point[1]);
	check(areaVector==correctAreaVector);
}

TestCase("ScalarStencilComputer compute average scalar stencil","[ScalarStencilComputer]")
{
	ScalarStencil scalarStencil[] = { {{0, 2.2},{4, 5.4}} , {{1, 4.8}, {4, 3.2}} };
	ScalarStencil correctScalarStencil = {{0, 1.1}, {1, 2.4}, {4, 4.3}};
	ScalarStencil averageScalarStencil = ScalarStencilComputer::computeAverageScalarStencil(scalarStencil[0],scalarStencil[1]);
	for(auto keyValue: averageScalarStencil)
		check(averageScalarStencil[keyValue.first]==Approx(correctScalarStencil[keyValue.first]));
}

TestCase("ScalarStencilComputer compute vector stencil","[ScalarStencilComputer]")
{
	Eigen::Vector3d point[] = { {0.0, 1.0, 0.0}, {-2.0, 0.0, 0.0} };
	ScalarStencil scalarStencil[] = { {{0, 2.2},{4, 5.4}} , {{1, 4.8}, {4, 3.2}} };
	VectorStencil vectorStencil = ScalarStencilComputer::computeVectorStencil(point[0], point[1], scalarStencil[0], scalarStencil[1]);
	Eigen::Vector3d areaVector = ScalarStencilComputer::computeAreaVector(point[0],point[1]);
	VectorStencil correctVectorStencil = 0.5 * (scalarStencil[0]+scalarStencil[1]) * areaVector;
	check(vectorStencil==correctVectorStencil);
}

TestCase("ScalarStencilComputer compute ScalarStencil on vertices", "[ScalarStencilComputer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
	std::vector<ScalarStencil> scalarStencilOnVertices = ScalarStencilComputer::inverseDistance(grid);
	std::vector<ScalarStencil> correctScalarStencilOnVertices = {
		{{0, 0.5}, {1, 0.5}},
		{{0, 1.0}},
		{{1, 1.0}},
		{{0, 0.5}, {1, 0.5}}
	};
	check(scalarStencilOnVertices==correctScalarStencilOnVertices);
}

TestCase("ScalarStencilComputer compute gradient using StaggeredQuadrangle","[ScalarStencilComputer][StaggeredQuadrangle]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
	// create staggered elements
	const unsigned staggeredQuadrangleIndex = 0;
	StaggeredQuadrangle staggeredQuadrangle(staggeredQuadrangleIndex,grid.vertices[0],grid.elements[0],grid.vertices[3],grid.elements[1]);
	const std::array<unsigned,4> staggeredTrianglesIndices = {{1, 2, 3, 4}};
	// compute vector stencil
	std::vector<ScalarStencil> scalarStencilOnVertices = ScalarStencilComputer::inverseDistance(grid);
	std::vector<ScalarStencil> scalarStencilOnElements = ScalarStencilComputer::elements(grid);
	VectorStencil correctVectorStencil = { { 0, {0.75,-0.75,0.0} }, { 1, {-0.75,0.75,0.0} } };
	VectorStencil vectorStencilOnStaggeredQuadrangle = ScalarStencilComputer::vectorStencil(staggeredQuadrangle, scalarStencilOnVertices, scalarStencilOnElements);
	for(auto& keyValue: vectorStencilOnStaggeredQuadrangle)
	{
		for(unsigned vectorEntry=0 ; vectorEntry<3 ; ++vectorEntry)
			check(keyValue.second[vectorEntry]==Approx(correctVectorStencil[keyValue.first][vectorEntry]));
	}
}

TestCase("ScalarStencilComputer compute gradient using StaggeredTriangle","[ScalarStencilComputer][StaggeredTriangle]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DVerticesWithNeighborElements grid(gridData);
	// create staggered elements
	constexpr unsigned numberOfTriangles = 4;
	const std::array<unsigned,numberOfTriangles> staggeredTrianglesIndices = {{1, 2, 3, 4}};
	std::array<StaggeredTriangle,numberOfTriangles> staggeredTriangles = {{
		{staggeredTrianglesIndices[0], grid.vertices[1], grid.elements[0], grid.vertices[0]},
		{staggeredTrianglesIndices[1], grid.vertices[3], grid.elements[0], grid.vertices[1]},
		{staggeredTrianglesIndices[2], grid.vertices[0], grid.elements[1], grid.vertices[2]},
		{staggeredTrianglesIndices[3], grid.vertices[2], grid.elements[1], grid.vertices[3]}
	}};
	// compute vector stencil
	std::vector<ScalarStencil> scalarStencilOnVertices = ScalarStencilComputer::inverseDistance(grid);
	std::vector<ScalarStencil> scalarStencilOnElements = ScalarStencilComputer::elements(grid);
	std::vector<VectorStencil> correctVectorStencil = {
		{ { 0, {0.25,0.25,0.0} }, { 1, {-0.25,-0.25,0.0} } },
		{ { 0, {-0.25,-0.25,0.0} }, { 1, {0.25,0.25,0.0} } },
		{ { 0, {-0.25,-0.25,0.0} }, { 1, {0.25,0.25,0.0} } },
		{ { 0, {0.25,0.25,0.0} }, { 1, {-0.25,-0.25,0.0} } }
	};
	for(unsigned triangleIndex=0 ; triangleIndex<numberOfTriangles ; ++triangleIndex)
	{
		VectorStencil vectorStencilOnStaggeredTriangle = ScalarStencilComputer::vectorStencil(staggeredTriangles[triangleIndex], scalarStencilOnVertices, scalarStencilOnElements);
		for(auto& keyValue: vectorStencilOnStaggeredTriangle)
		{
			for(unsigned vectorEntry=0 ; vectorEntry<3 ; ++vectorEntry)
				check(keyValue.second[vectorEntry]==Approx(correctVectorStencil[triangleIndex][keyValue.first][vectorEntry]));
		}
	}
}
