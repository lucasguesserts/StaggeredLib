#include <Utils/Test.hpp>
#include <vector>
#include <string>
#include <iostream>

#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Element2D.hpp>
#include <GeometricEntity/Line.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/Quadrangle.hpp>

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

TestCase("Element2D compute triangle area vector", "[Element2D]")
{
	std::vector<Vertex> vertices;
	vertices.push_back(Vertex(-5,  4,  7, 0));
	vertices.push_back(Vertex( 9, -2,  4, 1));
	vertices.push_back(Vertex( 3,  5, -6, 2));
	const Eigen::Vector3d areaVector(40.5, 79, 31);
	Triangle triangle;
	for(Vertex& vertex: vertices)
		triangle.addVertex(vertex);
	check(triangle.getAreaVector()==areaVector);
}

TestCase("Line", "[Element][Element2D][Line]")
{
	const unsigned numberOfVertices = 2;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex( 2.0, -5.0,  3.0, 0));
	 	vertices.push_back(Vertex( 4.0, -1.0,  6.0, 1));
	const Eigen::Vector3d localCoordinates(0.4, 0.0, 0.0);
	Line line;
	for(Vertex& vertex: vertices)
		line.addVertex(vertex);
	section("Basic requirements")
	{
		const unsigned dimension = 1;
		require(line.getNumberOfVertices()==numberOfVertices);
		require(line.dimension==dimension);
	}
	section("Centroid")
	{
		const Eigen::Vector3d centroid(3, -3, 4.5);
		check(line.getCentroid()==centroid);
	}
	section("Volume")
	{
		const double volume = 5.385164807134504; // the length
		check(line.getVolume()==Approx(volume));
	}
	section("Shape function values")
	{
		Eigen::Vector2d shapeFunctionValues(0.6, 0.4);
		Eigen::Vector2d lineShapeFunctionValues = line.getShapeFunctionValues(localCoordinates);
		for(unsigned i=0 ; i<2 ; ++i)
			check(lineShapeFunctionValues(i)==Approx(shapeFunctionValues(i)));
	}
	section("Shape function derivatives")
	{
		const unsigned numberOfRows = 2;
		const unsigned numberOfColumns = 3;
		Eigen::MatrixXd shapeFunctionDerivatives(numberOfRows, numberOfColumns);
			shapeFunctionDerivatives << -1.0, 0.0, 0.0,
		                                 1.0, 0.0, 0.0;
		Eigen::MatrixXd lineShapeFunctionDerivatives = line.getShapeFunctionDerivatives(localCoordinates);
		for(unsigned i=0 ; i<numberOfRows ; ++i)
			for(unsigned j=0 ; j<numberOfColumns ; ++j)
				check(lineShapeFunctionDerivatives(i,j)==Approx(shapeFunctionDerivatives(i,j)));
	}
}

TestCase("Triangle", "[Element][Element2D][Triangle]")
{
	const unsigned numberOfVertices = 3;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex(2.0, -5.0, 3.0, 0));
		vertices.push_back(Vertex(3.0, 7.0, -2.0, 1));
		vertices.push_back(Vertex(-4.0, -1.0, 6.0, 2));
	const Eigen::Vector3d localCoordinates(0.2, 0.3, 0.0);
	Triangle triangle;
	for(Vertex& vertex: vertices)
		triangle.addVertex(vertex);
	section("Basic requirements")
	{
		require(triangle.getNumberOfVertices()==3);
		require(triangle.dimension==2);
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
	section("Shape function values")
	{
		Eigen::VectorXd shapeFunctionValues(3); shapeFunctionValues << 0.5, 0.2, 0.3;
		check(triangle.getShapeFunctionValues(localCoordinates)==shapeFunctionValues);
	}
	section("Shape function derivatives")
	{
		Eigen::MatrixXd shapeFunctionDerivatives(3,3);
			shapeFunctionDerivatives << -1.0, -1.0, 0.0,
										 1.0,  0.0, 0.0,
										 0.0,  1.0, 0.0;
		check(triangle.getShapeFunctionDerivatives(localCoordinates)==shapeFunctionDerivatives);
	}
}

TestCase("Quadrangle", "[Element][Element2D][Quadrangle]")
{
	const unsigned numberOfVertices = 4;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex( 2.0, -5.0,  3.0, 0));
		vertices.push_back(Vertex( 4.0, -1.0,  6.0, 1));
		vertices.push_back(Vertex( 4.0,  3.0,  3.4, 2));
		vertices.push_back(Vertex( 3.0,  7.0, -2.0, 3));
	const Eigen::Vector3d localCoordinates(0.2, 0.3, 0.0);
	Quadrangle quadrangle;
	for(Vertex& vertex: vertices)
		quadrangle.addVertex(vertex);
	section("Basic requirements")
	{
		const unsigned dimension = 2;
		require(quadrangle.getNumberOfVertices()==numberOfVertices);
		require(quadrangle.dimension==dimension);
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
	section("Shape function values")
	{
		Eigen::Vector4d shapeFunctionValues(0.56, 0.14, 0.06, 0.24);
		Eigen::Vector4d quadrangleShapeFunctionValues = quadrangle.getShapeFunctionValues(localCoordinates);
		for(unsigned i=0 ; i<4 ; ++i)
			check(quadrangleShapeFunctionValues(i)==Approx(shapeFunctionValues(i)));
	}
	section("Shape function derivatives")
	{
		const unsigned numberOfRows = 4;
		const unsigned numberOfColumns = 3;
		Eigen::MatrixXd shapeFunctionDerivatives(numberOfRows, numberOfColumns);
			shapeFunctionDerivatives << -0.7, -0.8, 0.0,
		                                 0.7, -0.2, 0.0,
		                                 0.3,  0.2, 0.0,
		                                -0.3,  0.8, 0.0;
		Eigen::MatrixXd quadrangleShapeFunctionDerivatives = quadrangle.getShapeFunctionDerivatives(localCoordinates);
		for(unsigned i=0 ; i<numberOfRows ; ++i)
			for(unsigned j=0 ; j<numberOfColumns ; ++j)
				check(quadrangleShapeFunctionDerivatives(i,j)==Approx(shapeFunctionDerivatives(i,j)));
	}
}
