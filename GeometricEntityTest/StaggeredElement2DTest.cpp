#include <Utils/Test.hpp>
#include <vector>

#include <GeometricEntity/Test.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>

#include <Grid/GridData_2.hpp>
#include <Grid/Grid2D.hpp>

TestCase("Staggered element constructor", "[StaggeredElement2D]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData_2 gridData(cgnsFile);
	Grid2D grid(gridData);
	constexpr unsigned index = 14;
	Vertex &v0 = grid.vertices[0];
	Vertex &v1 = grid.vertices[3];
	Element *e0 = grid.elements[1];
	Element *e1 = grid.elements[3];
	section("Triangle")
	{
		StaggeredElement2D staggeredElement(index, v0, e0, v1);
		check(staggeredElement.vertices[0]==&v0);
		check(staggeredElement.elements[0]==e0);
		check(staggeredElement.vertices[1]==&v1);
		check(staggeredElement.elements[1]==nullptr);
	}
	section("Quadrangle")
	{
		StaggeredElement2D staggeredElement(index, v0, e0, v1, e1);
		check(staggeredElement.vertices[0]==&v0);
		check(staggeredElement.elements[0]==e0);
		check(staggeredElement.vertices[1]==&v1);
		check(staggeredElement.elements[1]==e1);
	}
}

TestCase("StaggeredElement2D operator==", "[StaggeredElement2D]")
{
    std::array<Vertex,3> vertices = {{
			{2.3, 5.8, 4.1, 6},
			{4.5, 1.6, 2.2, 47},
			{5.8, 3.4, 1.7, 65}
	}};
	std::array<Triangle,3> elements;
		elements[0].setIndex(0); elements[0].addVertex(vertices[0]); elements[0].addVertex(vertices[1]); elements[0].addVertex(vertices[2]);
		elements[1].setIndex(5); elements[1].addVertex(vertices[1]); elements[1].addVertex(vertices[2]); elements[1].addVertex(vertices[3]);
		elements[2].setIndex(7); elements[2].addVertex(vertices[2]); elements[2].addVertex(vertices[1]); elements[2].addVertex(vertices[0]);
	std::array<StaggeredElement2D,8> staggeredElements = {{
		{0, vertices[0], &elements[0], vertices[1], &elements[1]}, // reference
		{0, vertices[0], &elements[0], vertices[1], &elements[1]},
		{1, vertices[0], &elements[0], vertices[1], &elements[1]},
		{0, vertices[2], &elements[0], vertices[1], &elements[1]},
		{0, vertices[0], &elements[2], vertices[1], &elements[1]},
		{0, vertices[0], &elements[0], vertices[3], &elements[1]},
		{0, vertices[0], &elements[0], vertices[1], &elements[2]},
		{4, vertices[2], &elements[1], vertices[3], &elements[2]}
	}};
    section("true test")
    {
        check(staggeredElements[0]==staggeredElements[1]);
    }
    section("false tests")
    {
		checkFalse(staggeredElements[0]==staggeredElements[2]);
		checkFalse(staggeredElements[0]==staggeredElements[3]);
		checkFalse(staggeredElements[0]==staggeredElements[4]);
		checkFalse(staggeredElements[0]==staggeredElements[5]);
		checkFalse(staggeredElements[0]==staggeredElements[6]);
		checkFalse(staggeredElements[0]==staggeredElements[7]);
    }
}

TestCase("Staggered element 2D - triangle test", "[StaggeredElement2D]")
{
	constexpr unsigned index = 5;
	std::vector<Vertex> vertices = {
		{1.0, 1.0, 0.0, 5},
		{6.0, 3.0, 0.0, 14},
		{4.0, 5.0, 0.0, 19}
	};
	Triangle triangle;
		triangle.addVertex(vertices[0]);
		triangle.addVertex(vertices[1]);
		triangle.addVertex(vertices[2]);
	StaggeredElement2D staggeredElement(index, vertices[1], &triangle, vertices[0]);
	section("index")
	{
		check(staggeredElement.getIndex()==index);
	}
	section("vertices")
	{
		check(staggeredElement.vertices[0]==&vertices[1]);
		check(staggeredElement.vertices[1]==&vertices[0]);
	}
	section("element")
	{
		check(staggeredElement.elements[0]==&triangle);
	}
	section("centroid")
	{
		const Eigen::Vector3d centroid = {3.5, 2.0, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredElement.getCentroid()[i]==Approx(centroid[i]));
	}
	section("area vector")
	{
		const Eigen::Vector3d areaVector = {-2.0, 5.0, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredElement.getAreaVector()[i]==Approx(areaVector[i]));
	}
	section("volume")
	{
		constexpr double volume = 7.0/3.0;
		check(staggeredElement.getVolume()==Approx(volume));
	}
}

TestCase("Staggered element 2D - quadrangle test", "[StaggeredElement2D]")
{
	constexpr unsigned index = 5;
	std::vector<Vertex> vertices = {
		{1.0, 1.0, 0.0, 0},
		{6.0, 2.0, 0.0, 1},
		{4.0, 5.0, 0.0, 2},
		{2.0, 6.0, 0.0, 3},
		{-2.0, 4.0, 0.0, 4}
	};
	constexpr unsigned quadrangleIndex = 7;
	Quadrangle quadrangle;
		quadrangle.setIndex(quadrangleIndex);
		quadrangle.addVertex(vertices[0]);
		quadrangle.addVertex(vertices[1]);
		quadrangle.addVertex(vertices[2]);
		quadrangle.addVertex(vertices[3]);
	constexpr unsigned triangleIndex = 81;
	Triangle triangle;
		triangle.setIndex(triangleIndex);
		triangle.addVertex(vertices[0]);
		triangle.addVertex(vertices[3]);
		triangle.addVertex(vertices[4]);
	StaggeredElement2D staggeredElement(index, vertices[0], &quadrangle, vertices[3], &triangle);
	section("vertices")
	{
		check(staggeredElement.vertices[0]==&vertices[0]);
		check(staggeredElement.vertices[1]==&vertices[3]);
	}
	section("elements")
	{
		check(staggeredElement.elements[0]==&quadrangle);
		check(staggeredElement.elements[1]==&triangle);
	}
	section("centroid")
	{
		const Eigen::Vector3d centroid = {1.5, 3.5, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredElement.getCentroid()[i]==Approx(centroid[i]));
	}
	section("area vector")
	{
		const Eigen::Vector3d areaVector = {5.0, -1.0, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredElement.getAreaVector()[i]==Approx(areaVector[i]));
	}
	section("volume")
	{
		constexpr double volume = 7.375;
		check(staggeredElement.getVolume()==Approx(volume));
	}
}