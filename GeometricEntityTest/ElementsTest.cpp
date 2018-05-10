#include <Utils/Test.hpp>
#include <vector>

#include <Utils/EigenTest.hpp>
#include <GeometricEntity/Test.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Line.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/Quadrangle.hpp>

TestCase("Element compute triangle area vector", "[Element]")
{
	std::vector<Vertex> vertices;
	vertices.push_back(Vertex(-5,  4,  7, 0));
	vertices.push_back(Vertex( 9, -2,  4, 1));
	vertices.push_back(Vertex( 3,  5, -6, 2));
	const Eigen::Vector3d areaVector(40.5, 79, 31);
	check(Element::computeTriangleAreaVector(vertices[0],vertices[1],vertices[2])==areaVector);
}

TestCase("Line", "[Element][Line]")
{
	const unsigned numberOfVertices = 2;
	std::vector<Vertex> vertices;
		vertices.push_back(Vertex(2.0, -5.0, 0.0, 0));
		vertices.push_back(Vertex(3.0,  7.0, 0.0, 1));
	const Eigen::Vector3d localCoordinates(0.4, 0.0, 0.0);
	Line line;
	for(Vertex& vertex: vertices)
		line.addVertex(vertex);
	section("Basic requirements")
	{
		require(line.vertices.size()==2);
	}
	section("Centroid")
	{
		const Eigen::Vector3d centroid(5.0/2.0, 1.0, 0.0);
		check(line.getCentroid()==centroid);
	}
	section("Area vector")
	{
		const Eigen::Vector3d areaVector(12.0, -1.0, 0.0);
		check(line.getAreaVector()==areaVector);
	}
	section("Volume")
	{
		const double volume = 12.04159458; // the length
		check(line.getVolume()==Approx(volume));
	}
	section("Shape function values")
	{
		Eigen::VectorXd shapeFunctionValues(2);
		shapeFunctionValues << 0.6, 0.4;
		check(line.getShapeFunctionValues(localCoordinates)==shapeFunctionValues);
	}
	section("Shape function derivatives")
	{
		const unsigned numberOfRows = 2;
		const unsigned numberOfColumns = 3;
		Eigen::MatrixXd shapeFunctionDerivatives(numberOfRows, numberOfColumns);
			shapeFunctionDerivatives << -1.0, 0.0, 0.0,
		                                 1.0, 0.0, 0.0;
		check(line.getShapeFunctionDerivatives(localCoordinates)==shapeFunctionDerivatives);
	}
	section("positions matrix")
	{
		Eigen::MatrixXd positionsMatrix(3,numberOfVertices);
		for(unsigned vertex=0 ; vertex<numberOfVertices ; ++vertex)
		{
			positionsMatrix(0,vertex) = vertices[vertex].coeff(0);
			positionsMatrix(1,vertex) = vertices[vertex].coeff(1);
			positionsMatrix(2,vertex) = vertices[vertex].coeff(2);
		}
		check(line.getPositionsMatrix()==positionsMatrix);
	}
}

TestCase("Triangle", "[Element][Triangle]")
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
	section("Shape function values")
	{
		Eigen::VectorXd shapeFunctionValues(3);
		shapeFunctionValues << 0.5, 0.2, 0.3;
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
	section("positions matrix")
	{
		Eigen::MatrixXd positionsMatrix(3,numberOfVertices);
		for(unsigned vertex=0 ; vertex<numberOfVertices ; ++vertex)
		{
			positionsMatrix(0,vertex) = vertices[vertex].coeff(0);
			positionsMatrix(1,vertex) = vertices[vertex].coeff(1);
			positionsMatrix(2,vertex) = vertices[vertex].coeff(2);
		}
		check(triangle.getPositionsMatrix()==positionsMatrix);
	}
}

TestCase("Line operator==", "[Line]")
{
    std::array<Vertex,3> vertices = {{
			{2.3, 5.8, 4.1, 6},
			{2.8, 4.9, 5.1, 32},
			{4.5, 1.6, 2.2, 47}
	}};
	std::array<Line,5> lines;
		lines[0].setIndex(0); lines[0].addVertex(vertices[0]); lines[0].addVertex(vertices[1]); //reference
		lines[1].setIndex(0); lines[1].addVertex(vertices[0]); lines[1].addVertex(vertices[1]);
		lines[2].setIndex(4); lines[2].addVertex(vertices[0]); lines[2].addVertex(vertices[1]);
		lines[3].setIndex(0); lines[3].addVertex(vertices[3]); lines[3].addVertex(vertices[1]);
		lines[4].setIndex(0); lines[4].addVertex(vertices[0]); lines[4].addVertex(vertices[2]);
	section("true test")
	{
		check(lines[0]==lines[1]);
	}
	section("false tests")
	{
		checkFalse(lines[0]==lines[2]);
		checkFalse(lines[0]==lines[3]);
		checkFalse(lines[0]==lines[4]);
	}
}

TestCase("Triangle operator==", "[Triangle]")
{
    std::array<Vertex,4> vertices = {{
			{2.3, 5.8, 4.1, 6},
			{2.8, 4.9, 5.1, 32},
			{4.5, 1.6, 2.2, 47},
			{5.8, 3.4, 1.7, 65}
	}};
	std::array<Triangle,6> triangles;
		triangles[0].setIndex(0); triangles[0].addVertex(vertices[0]); triangles[0].addVertex(vertices[1]); triangles[0].addVertex(vertices[2]); //reference
		triangles[1].setIndex(0); triangles[1].addVertex(vertices[0]); triangles[1].addVertex(vertices[1]); triangles[1].addVertex(vertices[2]);
		triangles[2].setIndex(4); triangles[2].addVertex(vertices[0]); triangles[2].addVertex(vertices[1]); triangles[2].addVertex(vertices[2]);
		triangles[3].setIndex(0); triangles[3].addVertex(vertices[3]); triangles[3].addVertex(vertices[1]); triangles[3].addVertex(vertices[2]);
		triangles[4].setIndex(0); triangles[4].addVertex(vertices[0]); triangles[4].addVertex(vertices[3]); triangles[4].addVertex(vertices[2]);
		triangles[5].setIndex(0); triangles[5].addVertex(vertices[0]); triangles[5].addVertex(vertices[1]); triangles[5].addVertex(vertices[3]);
	section("true test")
	{
		check(triangles[0]==triangles[1]);
	}
	section("false tests")
	{
		checkFalse(triangles[0]==triangles[2]);
		checkFalse(triangles[0]==triangles[3]);
		checkFalse(triangles[0]==triangles[4]);
		checkFalse(triangles[0]==triangles[5]);
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
	const Eigen::Vector3d localCoordinates(0.2, 0.3, 0.0);
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

	section("Shape function values")
	{
		Eigen::VectorXd shapeFunctionValues(4);
		shapeFunctionValues << 0.56, 0.14, 0.06, 0.24;
		check(quadrangle.getShapeFunctionValues(localCoordinates)==shapeFunctionValues);
	}
	section("Shape function derivatives")
	{
		Eigen::MatrixXd shapeFunctionDerivatives(4, 3);
		shapeFunctionDerivatives << -0.7, -0.8, 0.0,
									0.7, -0.2, 0.0,
									0.3,  0.2, 0.0,
									-0.3,  0.8, 0.0;
		check(quadrangle.getShapeFunctionDerivatives(localCoordinates)==shapeFunctionDerivatives);
	}
	section("positions matrix")
	{
		Eigen::MatrixXd positionsMatrix(3,numberOfVertices);
		for(unsigned vertex=0 ; vertex<numberOfVertices ; ++vertex)
		{
			positionsMatrix(0,vertex) = vertices[vertex].coeff(0);
			positionsMatrix(1,vertex) = vertices[vertex].coeff(1);
			positionsMatrix(2,vertex) = vertices[vertex].coeff(2);
		}
		check(quadrangle.getPositionsMatrix()==positionsMatrix);
	}
}

TestCase("Quadrangle operator==", "[Quadrangle]")
{
    std::array<Vertex,5> vertices = {{
			{2.3, 5.8, 4.1, 6},
			{2.8, 4.9, 5.1, 32},
			{4.5, 1.6, 2.2, 47},
			{5.8, 3.4, 1.7, 65},
			{7.3, 9.4, 8.8, 26}
	}};
	std::array<Quadrangle,7> quadrangles;
		quadrangles[0].setIndex(0); quadrangles[0].addVertex(vertices[0]); quadrangles[0].addVertex(vertices[1]); quadrangles[0].addVertex(vertices[2]), quadrangles[0].addVertex(vertices[3]); //reference
		quadrangles[1].setIndex(0); quadrangles[1].addVertex(vertices[0]); quadrangles[1].addVertex(vertices[1]); quadrangles[1].addVertex(vertices[2]), quadrangles[1].addVertex(vertices[3]);
		quadrangles[2].setIndex(4); quadrangles[2].addVertex(vertices[0]); quadrangles[2].addVertex(vertices[1]); quadrangles[2].addVertex(vertices[2]), quadrangles[2].addVertex(vertices[3]);
		quadrangles[3].setIndex(0); quadrangles[3].addVertex(vertices[4]); quadrangles[3].addVertex(vertices[1]); quadrangles[3].addVertex(vertices[2]), quadrangles[3].addVertex(vertices[3]);
		quadrangles[4].setIndex(0); quadrangles[4].addVertex(vertices[0]); quadrangles[4].addVertex(vertices[4]); quadrangles[4].addVertex(vertices[2]), quadrangles[4].addVertex(vertices[3]);
		quadrangles[5].setIndex(0); quadrangles[5].addVertex(vertices[0]); quadrangles[5].addVertex(vertices[1]); quadrangles[5].addVertex(vertices[4]), quadrangles[5].addVertex(vertices[3]);
		quadrangles[5].setIndex(0); quadrangles[5].addVertex(vertices[0]); quadrangles[5].addVertex(vertices[1]); quadrangles[5].addVertex(vertices[2]), quadrangles[6].addVertex(vertices[4]);
	section("true test")
	{
		check(quadrangles[0]==quadrangles[1]);
	}
	section("false tests")
	{
		checkFalse(quadrangles[0]==quadrangles[2]);
		checkFalse(quadrangles[0]==quadrangles[3]);
		checkFalse(quadrangles[0]==quadrangles[4]);
		checkFalse(quadrangles[0]==quadrangles[5]);
		checkFalse(quadrangles[0]==quadrangles[6]);
	}
}