#include <Grid/Grid2D.hpp>

Grid2D::Grid2D(const GridData& gridData)
	: Grid(gridData)
{
	if(gridData.dimension==2)
	{
		this->dimension = 2;
		this->buildLinesOn2DGrid(gridData);
		this->buildTrianglesOn2DGrid(gridData);
		this->buildQuadranglesOn2DGrid(gridData);
		this->assignElementsPointers();
	}
}

void Grid2D::buildLinesOn2DGrid(const GridData& gridData)
{
	constexpr unsigned numberOfVerticesPerLine = 2;
	const unsigned numberOfLines = gridData.line.size();
	this->lines.reserve(numberOfLines);
	for(const ElementDefinition<2>& lineDefinition: gridData.line)
	{
		Line line;
		line.setIndex(lineDefinition.index);
		for(unsigned vertexLocalIndexInLine=0 ; vertexLocalIndexInLine<numberOfVerticesPerLine ; ++vertexLocalIndexInLine)
		{
			unsigned vertexIndex = lineDefinition.connectivity(vertexLocalIndexInLine);
			line.addVertex(this->vertices[vertexIndex]);
		}
		this->lines.push_back(line);
	}
	return;
}

void Grid2D::buildTrianglesOn2DGrid(const GridData& gridData)
{
	constexpr unsigned numberOfVerticesPerTriangle = 3;
	const unsigned numberOfTriangles = gridData.triangle.size();
	this->triangles.reserve(numberOfTriangles);
	for(const ElementDefinition<3>& triangleDefinition: gridData.triangle)
	{
		Triangle triangle;
		triangle.setIndex(triangleDefinition.index);
		for(unsigned vertexLocalIndexInTriangle=0 ; vertexLocalIndexInTriangle<numberOfVerticesPerTriangle ; ++vertexLocalIndexInTriangle)
		{
			unsigned vertexIndex = triangleDefinition.connectivity(vertexLocalIndexInTriangle);
			triangle.addVertex(this->vertices[vertexIndex]);
		}
		this->triangles.push_back(triangle);
	}
	return;
}

void Grid2D::buildQuadranglesOn2DGrid(const GridData& gridData)
{
	constexpr unsigned numberOfVerticesPerQuadrangle = 4;
	const unsigned numberOfQuadrangles = gridData.quadrangle.size();
	this->quadrangles.reserve(numberOfQuadrangles);
	for(const ElementDefinition<4>& quadrangleDefinition: gridData.quadrangle)
	{
		Quadrangle quadrangle;
		quadrangle.setIndex(quadrangleDefinition.index);
		for(unsigned vertexLocalIndexInQuadrangle=0 ; vertexLocalIndexInQuadrangle<numberOfVerticesPerQuadrangle ; ++vertexLocalIndexInQuadrangle)
		{
			unsigned vertexIndex = quadrangleDefinition.connectivity(vertexLocalIndexInQuadrangle);
			quadrangle.addVertex(this->vertices[vertexIndex]);
		}
		this->quadrangles.push_back(quadrangle);
	}
	return;
}

void Grid2D::assignElementsPointers()
{
	const unsigned numberOfElements = this->triangles.size() + this->quadrangles.size();
	this->elements.resize(numberOfElements);
	for(Triangle& triangle: this->triangles)
		this->elements[triangle.getIndex()] = (Element*) &triangle;
	for(Quadrangle& quadrangle: this->quadrangles)
		this->elements[quadrangle.getIndex()] = (Element*) &quadrangle;
	return;
}
