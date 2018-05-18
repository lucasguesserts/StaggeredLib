#include <Grid/Grid2D.hpp>

Grid2D::Grid2D(const GridData_2& gridData)
	: Grid(gridData)
{
	if(gridData.dimension==2)
	{
		this->dimension = 2;
		this->buildLinesOn2DGrid_2(gridData);
		this->buildTrianglesOn2DGrid_2(gridData);
		this->buildQuadranglesOn2DGrid_2(gridData);
		this->assignElementsPointers();
	}
}

Grid2D::Grid2D(const std::string& fileName)
	: Grid(fileName)
{
		this->dimension = 2;
		this->buildLinesOn2DGrid();
		this->buildTrianglesOn2DGrid();
		this->buildQuadranglesOn2DGrid();
		this->assignElementsPointers();
	return;
}

void Grid2D::buildLinesOn2DGrid(void)
{
	const std::vector<std::array<int, 3>>& lineConnectivity = this->cgnsReader->gridData->lineConnectivity;
	this->lines.reserve(lineConnectivity.size());
	for(auto& lineDefinition: lineConnectivity)
	{
		Line line;
		line.setIndex(lineDefinition.back());
		line.addVertex(this->vertices[lineDefinition[0]]);
		line.addVertex(this->vertices[lineDefinition[1]]);
		this->lines.push_back(line);
	}
	return;
}

void Grid2D::buildLinesOn2DGrid_2(const GridData_2& gridData)
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

void Grid2D::buildTrianglesOn2DGrid(void)
{
	const std::vector<std::array<int,4>>& triangleConnectivity = this->cgnsReader->gridData->triangleConnectivity;
	this->triangles.reserve(triangleConnectivity.size());
	for(auto& triangleDefinition: triangleConnectivity)
	{
		Triangle triangle;
		triangle.setIndex(triangleDefinition.back());
		triangle.addVertex(this->vertices[triangleDefinition[0]]);
		triangle.addVertex(this->vertices[triangleDefinition[1]]);
		triangle.addVertex(this->vertices[triangleDefinition[2]]);
		this->triangles.push_back(triangle);
	}
	return;
}

void Grid2D::buildTrianglesOn2DGrid_2(const GridData_2& gridData)
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

void Grid2D::buildQuadranglesOn2DGrid(void)
{
	const std::vector<std::array<int,5>>& quadrangleConnectivity = this->cgnsReader->gridData->quadrangleConnectivity;
	this->quadrangles.reserve(quadrangleConnectivity.size());
	for(auto& quadrangleDefinition: quadrangleConnectivity)
	{
		Quadrangle quadrangle;
		quadrangle.setIndex(quadrangleDefinition.back());
		quadrangle.addVertex(this->vertices[quadrangleDefinition[0]]);
		quadrangle.addVertex(this->vertices[quadrangleDefinition[1]]);
		quadrangle.addVertex(this->vertices[quadrangleDefinition[2]]);
		quadrangle.addVertex(this->vertices[quadrangleDefinition[3]]);
		this->quadrangles.push_back(quadrangle);
	}
	return;
}

void Grid2D::buildQuadranglesOn2DGrid_2(const GridData_2& gridData)
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
