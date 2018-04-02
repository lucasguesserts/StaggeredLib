#include <Utils/Test.hpp>
#include <vector>

#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/Quadrangle.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>

TestCase("Element compute triangle area vector", "[Element]")
{
	std::vector<Vertex> vertices;
	vertices.push_back(Vertex(-5,  4,  7, 0));
	vertices.push_back(Vertex( 9, -2,  4, 1));
	vertices.push_back(Vertex( 3,  5, -6, 2));
	const Eigen::Vector3d areaVector(40.5, 79, 31);
	check(Element::computeTriangleAreaVector(vertices[0],vertices[1],vertices[2])==areaVector);
}

TestCase("Triangle", "[Element][Triangle]")
{
	const unsigned numberOfVertices = 3;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex(2.0, -5.0, 3.0, 0));
		vertices.push_back(Vertex(3.0, 7.0, -2.0, 1));
		vertices.push_back(Vertex(-4.0, -1.0, 6.0, 2));
	Triangle triangle;
	for(Vertex& vertex: vertices)
		triangle.addVertex(vertex);
	section("Basic requirements")
	{
		require(triangle.vertices.size()==3);
	}
	section("Centroid")
	{
		const Eigen::Vector3d centroid(1.0/3.0, 1.0/3.0, 7.0/3.0);
		check(triangle.getCentroid()==centroid);
	}
	section("Area vector")
	{
		const Eigen::Vector3d areaVector(28.0, 13.5, 38.0);
		check(triangle.getAreaVector()==areaVector);
	}
	section("Volume")
	{
		const double volume = 49.09429702; // the area
		check(triangle.getVolume()==Approx(volume));
	}
}

TestCase("Quadrangle", "[Element][Quadrangle]")
{
	const unsigned numberOfVertices = 4;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex( 2.0, -5.0,  3.0, 0));
		vertices.push_back(Vertex( 4.0, -1.0,  6.0, 1));
		vertices.push_back(Vertex( 4.0,  3.0,  3.4, 2));
		vertices.push_back(Vertex( 3.0,  7.0, -2.0, 3));
	Quadrangle quadrangle;
	for(Vertex& vertex: vertices)
		quadrangle.addVertex(vertex);
	section("Basic requirements")
	{
		require(quadrangle.vertices.size()==numberOfVertices);
	}
	section("Centroid")
	{
		const Eigen::Vector3d centroid(3.25, 1.0, 2.6);
		check(quadrangle.getCentroid()==centroid);
	}
	section("Area vector")
	{
		const Eigen::Vector3d areaVector(-33.6, 7.8, 12);
		Eigen::Vector3d quadrangleAreaVector = quadrangle.getAreaVector();
		for(unsigned i=0 ; i<3 ; ++i)
			check(quadrangleAreaVector(i)==Approx(areaVector(i)));
	}
	section("Volume")
	{
		const double volume = 36.5212267044797; // the area
		check(quadrangle.getVolume()==Approx(volume));
	}
}

TestCase("StaggeredTriangle operator==", "[StaggeredTriangle]")
{
    std::array<Vertex,3> vertices = {{
			{2.3, 5.8, 4.1, 6},
			{4.5, 1.6, 2.2, 47},
			{5.8, 3.4, 1.7, 65}
	}};
	std::array<Triangle,2> elements;
		elements[0].setIndex(0); elements[0].addVertex(vertices[0]); elements[0].addVertex(vertices[1]); elements[0].addVertex(vertices[2]);
		elements[1].setIndex(5); elements[1].addVertex(vertices[1]); elements[1].addVertex(vertices[2]); elements[1].addVertex(vertices[3]);
	std::array<StaggeredTriangle,7> staggeredTriangles = {{
		{0, vertices[0], &elements[0], vertices[1]}, // reference
		{0, vertices[0], &elements[0], vertices[1]},
		{1, vertices[0], &elements[0], vertices[1]},
		{0, vertices[2], &elements[0], vertices[1]},
		{0, vertices[0], &elements[2], vertices[1]},
		{0, vertices[0], &elements[0], vertices[3]},
		{4, vertices[2], &elements[1], vertices[3]}
	}};
    section("true test")
    {
        check(staggeredTriangles[0]==staggeredTriangles[1]);
    }
    section("false tests")
    {
		checkFalse(staggeredTriangles[0]==staggeredTriangles[2]);
		checkFalse(staggeredTriangles[0]==staggeredTriangles[3]);
		checkFalse(staggeredTriangles[0]==staggeredTriangles[4]);
		checkFalse(staggeredTriangles[0]==staggeredTriangles[5]);
		checkFalse(staggeredTriangles[0]==staggeredTriangles[6]);
    }
}

TestCase("staggered triangle member functions", "[StaggeredTriangle]")
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
	StaggeredTriangle staggeredTriangle(index, vertices[1], &triangle, vertices[0]);
	section("index")
	{
		check(staggeredTriangle.getIndex()==index);
	}
	section("vertices")
	{
		check(staggeredTriangle.vertices[0]==&vertices[1]);
		check(staggeredTriangle.vertices[1]==&vertices[0]);
	}
	section("element")
	{
		check(staggeredTriangle.element==&triangle);
	}
	section("centroid")
	{
		const Eigen::Vector3d centroid = {3.5, 2.0, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredTriangle.getCentroid()[i]==Approx(centroid[i]));
	}
	section("area vector")
	{
		const Eigen::Vector3d areaVector = {-2.0, 5.0, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredTriangle.getAreaVector()[i]==Approx(areaVector[i]));
	}
	section("volume")
	{
		constexpr double volume = 7.0/3.0;
		check(staggeredTriangle.getVolume()==Approx(volume));
	}
}
TestCase("StaggeredQuadrangle operator==", "[StaggeredQuadrangle]")
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
	std::array<StaggeredQuadrangle,8> staggeredQuadrangles = {{
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
        check(staggeredQuadrangles[0]==staggeredQuadrangles[1]);
    }
    section("false tests")
    {
		checkFalse(staggeredQuadrangles[0]==staggeredQuadrangles[2]);
		checkFalse(staggeredQuadrangles[0]==staggeredQuadrangles[3]);
		checkFalse(staggeredQuadrangles[0]==staggeredQuadrangles[4]);
		checkFalse(staggeredQuadrangles[0]==staggeredQuadrangles[5]);
		checkFalse(staggeredQuadrangles[0]==staggeredQuadrangles[6]);
		checkFalse(staggeredQuadrangles[0]==staggeredQuadrangles[7]);
    }
}

TestCase("staggered quadrangle", "[StaggeredQuadrangle]")
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
	StaggeredQuadrangle staggeredQuadrangle(index, vertices[0], &quadrangle, vertices[3], &triangle);
	section("vertices")
	{
		check(staggeredQuadrangle.vertices[0]==&vertices[0]);
		check(staggeredQuadrangle.vertices[1]==&vertices[3]);
	}
	section("elements")
	{
		check(staggeredQuadrangle.elements[0]==&quadrangle);
		check(staggeredQuadrangle.elements[1]==&triangle);
	}
	section("centroid")
	{
		const Eigen::Vector3d centroid = {1.5, 3.5, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredQuadrangle.getCentroid()[i]==Approx(centroid[i]));
	}
	section("area vector")
	{
		const Eigen::Vector3d areaVector = {5.0, -1.0, 0.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredQuadrangle.getAreaVector()[i]==Approx(areaVector[i]));
	}
	section("volume")
	{
		constexpr double volume = 7.375;
		check(staggeredQuadrangle.getVolume()==Approx(volume));
	}
}