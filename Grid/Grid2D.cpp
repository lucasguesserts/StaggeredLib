#include <Grid/Grid2D.hpp>

Grid2D::Grid2D(const GridData& gridData)
	: Grid(gridData)
{
	if(gridData.dimension==2)
	{
		this->dimension = 2;
		this->buildTrianglesOn2DGrid(gridData);
		this->buildQuadranglesOn2DGrid(gridData);
		this->assignElementsPointers();
	}
}

void Grid2D::buildTrianglesOn2DGrid(const GridData& gridData)
{
	const unsigned numberOfVerticesPerTriangle = 3;
	const unsigned numberOfTriangles = gridData.triangleConnectivity.rows();
	this->triangles.reserve(numberOfTriangles);
	for(unsigned triangleIndex=0 ; triangleIndex<numberOfTriangles ; ++triangleIndex)
	{
		Triangle triangle;
		for(unsigned vertexLocalIndexInTriangle=0 ; vertexLocalIndexInTriangle<numberOfVerticesPerTriangle ; ++vertexLocalIndexInTriangle)
		{
			unsigned vertexIndex = gridData.triangleConnectivity(triangleIndex,vertexLocalIndexInTriangle);
			triangle.addVertex(this->vertices[vertexIndex]);
		}
		this->triangles.push_back(triangle);
	}
	return ;
}

void Grid2D::buildQuadranglesOn2DGrid(const GridData& gridData)
{
	const unsigned numberOfVerticesPerQuadrangle = 4;
	const unsigned numberOfQuadrangles = gridData.quadrangleConnectivity.rows();
	this->quadrangles.reserve(numberOfQuadrangles);
	for(unsigned quadrangleIndex=0 ; quadrangleIndex<numberOfQuadrangles ; ++quadrangleIndex)
	{
		Quadrangle quadrangle;
		for(unsigned vertexLocalIndexInQuadrangle=0 ; vertexLocalIndexInQuadrangle<numberOfVerticesPerQuadrangle ; ++vertexLocalIndexInQuadrangle)
		{
			unsigned vertexIndex = gridData.quadrangleConnectivity(quadrangleIndex,vertexLocalIndexInQuadrangle);
			quadrangle.addVertex(this->vertices[vertexIndex]);
		}
		this->quadrangles.push_back(quadrangle);
	}
	return ;
}

void Grid2D::assignElementsPointers()
{
	for(Triangle& triangle: this->triangles)
		this->elements.push_back((Element*) &triangle);
	for(Quadrangle& quadrangle: this->quadrangles)
		this->elements.push_back((Element*) &quadrangle);
	return ;
}
