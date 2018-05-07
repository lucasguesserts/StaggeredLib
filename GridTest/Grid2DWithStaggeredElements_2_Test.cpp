#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <vector>
#include <string>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>

#include <GeometricEntity/Test.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredElement.hpp>

#include <Grid/Grid2DWithStaggeredElements_2.hpp>

TestCase("staggered elements with the same vertices", "[Grid2DWithStaggeredElements_2]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements_2 grid(gridData);
	std::vector<StaggeredElement> staggeredElements = {
		{14, grid.vertices[0], grid.elements[1], grid.vertices[1]},
		{14, grid.vertices[0], grid.elements[1], grid.vertices[1]}, //true
		{29, grid.vertices[0], grid.elements[1], grid.vertices[1]}, //true
		{14, grid.vertices[0], grid.elements[0], grid.vertices[1]}, //true
		{14, grid.vertices[0], grid.elements[1], grid.vertices[1], grid.elements[0]}, //true
		{53, grid.vertices[2], grid.elements[0], grid.vertices[1], grid.elements[0]}, //false
		{53, grid.vertices[0], grid.elements[0], grid.vertices[3], grid.elements[0]}, //false
		{53, grid.vertices[2], grid.elements[0], grid.vertices[3], grid.elements[0]}, //false
	};
	section("true tests")
	{
		check(grid.staggeredElementsHaveTheSameVertices(staggeredElements[0],staggeredElements[1]));
		check(grid.staggeredElementsHaveTheSameVertices(staggeredElements[0],staggeredElements[2]));
		check(grid.staggeredElementsHaveTheSameVertices(staggeredElements[0],staggeredElements[3]));
		check(grid.staggeredElementsHaveTheSameVertices(staggeredElements[0],staggeredElements[4]));
	}
	section("false tests")
	{
		checkFalse(grid.staggeredElementsHaveTheSameVertices(staggeredElements[0],staggeredElements[5]));
		checkFalse(grid.staggeredElementsHaveTheSameVertices(staggeredElements[0],staggeredElements[6]));
		checkFalse(grid.staggeredElementsHaveTheSameVertices(staggeredElements[0],staggeredElements[7]));
	}
}

TestCase("Find staggered element in Grid2D", "[Grid2DWithStaggeredElements_2]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements_2 grid(gridData);
	std::vector<StaggeredElement> staggeredElements = {
		{0, grid.vertices[1], grid.elements[0], grid.vertices[0]},
		{1, grid.vertices[1], grid.elements[0], grid.vertices[0]},
		{0, grid.vertices[1], grid.elements[1], grid.vertices[0]},
		{0, grid.vertices[0], grid.elements[0], grid.vertices[1]},
		{2, grid.vertices[3], grid.elements[1], grid.vertices[0], grid.elements[0]},
		{2, grid.vertices[3], grid.elements[1], grid.vertices[0], grid.elements[0]},
		{5, grid.vertices[3], grid.elements[0], grid.vertices[0]},
		{0, grid.vertices[1], grid.elements[0], grid.vertices[2]},
		{5, grid.vertices[2], grid.elements[0], grid.vertices[1]}
	};
	section("true tests")
	{
		std::tuple<bool,unsigned> location;
		location = grid.findStaggeredElement(staggeredElements[0]);
			check(std::get<bool>(location));
			check(std::get<unsigned>(location)==0);
		location = grid.findStaggeredElement(staggeredElements[1]);
			check(std::get<bool>(location));
			check(std::get<unsigned>(location)==0);
		location = grid.findStaggeredElement(staggeredElements[2]);
			check(std::get<bool>(location));
			check(std::get<unsigned>(location)==0);
		location = grid.findStaggeredElement(staggeredElements[3]);
			check(std::get<bool>(location));
			check(std::get<unsigned>(location)==0);
		location = grid.findStaggeredElement(staggeredElements[4]);
			check(std::get<bool>(location));
			check(std::get<unsigned>(location)==2);
		location = grid.findStaggeredElement(staggeredElements[5]);
			check(std::get<bool>(location));
			check(std::get<unsigned>(location)==2);
		location = grid.findStaggeredElement(staggeredElements[6]);
			check(std::get<bool>(location));
			check(std::get<unsigned>(location)==2);
	}
	section("false tests")
	{
		checkFalse(std::get<bool>(grid.findStaggeredElement(staggeredElements[7])));
		checkFalse(std::get<bool>(grid.findStaggeredElement(staggeredElements[8])));
	}
}

TestCase("Organize staggered elements for face creation", "[Grid2DWithStaggeredElements_2]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements_2 grid(gridData);
	section("element 0 and vertex 0")
	{
		Vertex * vertex = &grid.vertices[0];
		Element* element = grid.elements[0];
		section("swap")
		{
			StaggeredElement* front = &grid.staggeredElements[0];
			StaggeredElement* back = &grid.staggeredElements[2];
			grid.organizeStaggeredElements(element, vertex, back, front);
			check(front==&grid.staggeredElements[2]);
			check(back==&grid.staggeredElements[0]);
		}
		section("do not swap")
		{
			StaggeredElement* front = &grid.staggeredElements[2];
			StaggeredElement* back = &grid.staggeredElements[0];
			grid.organizeStaggeredElements(element, vertex, back, front);
			check(front==&grid.staggeredElements[2]);
			check(back==&grid.staggeredElements[0]);
		}
	}
	section("element 1 and vertex 2")
	{
		Vertex * vertex = &grid.vertices[2];
		Element* element = grid.elements[1];
		section("swap")
		{
			StaggeredElement* front = &grid.staggeredElements[4];
			StaggeredElement* back = &grid.staggeredElements[3];
			grid.organizeStaggeredElements(element, vertex, back, front);
			check(front==&grid.staggeredElements[3]);
			check(back==&grid.staggeredElements[4]);
		}
		section("do not swap")
		{
			StaggeredElement* front = &grid.staggeredElements[3];
			StaggeredElement* back = &grid.staggeredElements[4];
			grid.organizeStaggeredElements(element, vertex, back, front);
			check(front==&grid.staggeredElements[3]);
			check(back==&grid.staggeredElements[4]);
		}
	}
}

TestCase("Grid2DWithStaggeredElements_2 build", "[Grid2DWithStaggeredElements_2]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements_2 grid(gridData);
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
	section("staggered elements")
	{
		std::vector<StaggeredElement> staggeredElements = {
			{0, grid.vertices[1], grid.elements[0], grid.vertices[0]},
			{1, grid.vertices[3], grid.elements[0], grid.vertices[1]},
			{2, grid.vertices[0], grid.elements[0], grid.vertices[3], grid.elements[1]},
			{3, grid.vertices[2], grid.elements[1], grid.vertices[3]},
			{4, grid.vertices[0], grid.elements[1], grid.vertices[2]}
		};
		check(grid.staggeredElements==staggeredElements);
	}
	section("faces")
	{
		std::vector<Face> faces = {
			{0, 0, *(grid.elements[0]), grid.vertices[0], grid.staggeredElements[0], grid.staggeredElements[2]},
			{1, 1, *(grid.elements[0]), grid.vertices[1], grid.staggeredElements[1], grid.staggeredElements[0]},
			{2, 2, *(grid.elements[0]), grid.vertices[3], grid.staggeredElements[2], grid.staggeredElements[1]},
			{3, 0, *(grid.elements[1]), grid.vertices[0], grid.staggeredElements[2], grid.staggeredElements[4]},
			{4, 1, *(grid.elements[1]), grid.vertices[3], grid.staggeredElements[3], grid.staggeredElements[2]},
			{5, 2, *(grid.elements[1]), grid.vertices[2], grid.staggeredElements[4], grid.staggeredElements[3]}
		};
		check(grid.faces==faces);
	}
}