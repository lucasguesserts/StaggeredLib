#include <Utils/Test.hpp>
#include <vector>
#include <string>
#include <iostream>

#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/Quadrangle.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>

TestCase("Entity", "[Entity]")
{
	const unsigned index = 3;
	section("constructor")
	{
		Entity entity(index);
		check(entity.getIndex()==index);
	}
	section("set index")
	{
		Entity entity;
		entity.setIndex(index);
		check(entity.getIndex()==index);
	}
}

TestCase("Vertex constructor", "[Vertex]")
{
	const std::vector<double> values = {3.2, -5.7, 9.4};
	const unsigned index = 4;
	Vertex vertex(values[0], values[1], values[2], index);
	section("Index")
	{
		check(vertex.getIndex()==index);
		const unsigned newIndex = 7;
		vertex.setIndex(newIndex);
		check(vertex.getIndex()==newIndex);
	}
	section("constructor values")
	{
		for(unsigned i=0 ; i<3 ; ++i)
			check(vertex(i)==values[i]);
	}
	section("Eigen::Vector functionalities")
	{
		Eigen::Vector3d eigenVector(4.2, 9.3, -6.1);
		Eigen::Vector3d sumAnswer(7.4, 3.6, 3.3);
		Eigen::Vector3d sum = vertex + eigenVector;
		for(unsigned i=0 ; i<3 ; ++i)
		check(sum[i]==Approx(sumAnswer[i]));
	}
}

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

TestCase("staggered triangle", "[StaggeredTriangle]")
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
		const Eigen::Vector3d areaVector = {0.0, 0.0, 7.0/3.0};
		for(unsigned i=0 ; i<3 ; ++i)
			check(staggeredTriangle.getAreaVector()[i]==Approx(areaVector[i]));
	}
	section("volume")
	{
		constexpr double volume = 7.0/3.0;
		check(staggeredTriangle.getVolume()==Approx(volume));
	}
}
