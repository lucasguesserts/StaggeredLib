#include <Grid/GridBuilder.hpp>
#include <Grid/Vertex.hpp>

Grid2D GridBuilder::build2D(const GridData gridData)
{
	Grid2D grid2D;
	if(GridBuilder::gridIs2D(gridData))
	{
		grid2D.dimension = 2;
		GridBuilder::buildVerticesOn2DGrid(gridData, grid2D);
		GridBuilder::buildTrianglesOn2DGrid(gridData, grid2D);
		GridBuilder::buildQuadranglesOn2DGrid(gridData, grid2D);
		GridBuilder::assignElementsPointers(grid2D);
	}
	return grid2D;
}

bool GridBuilder::gridIs2D(const GridData& gridData)
{
	return gridData.dimension==2;
}

void GridBuilder::buildVerticesOn2DGrid(const GridData& gridData, Grid2D& grid2D)
{
	const unsigned numberOfVertices = gridData.coordinates.rows();
	grid2D.vertices.reserve(numberOfVertices);
	for(unsigned vertexIndex=0 ; vertexIndex<numberOfVertices ; ++vertexIndex)
	{
		const double x = gridData.coordinates(vertexIndex,0);
		const double y = gridData.coordinates(vertexIndex,1);
		const double z = gridData.coordinates(vertexIndex,2);
		grid2D.vertices.push_back(Vertex(x, y, z, vertexIndex));
	}
	return ;
}

void GridBuilder::buildTrianglesOn2DGrid(const GridData& gridData, Grid2D& grid2D)
{
	const unsigned numberOfVerticesPerTriangle = 3;
	const unsigned numberOfTriangles = gridData.triangleConnectivity.rows();
	grid2D.triangles.reserve(numberOfTriangles);
	for(unsigned triangleIndex=0 ; triangleIndex<numberOfTriangles ; ++triangleIndex)
	{
		Triangle triangle;
		for(unsigned vertexLocalIndexInTriangle=0 ; vertexLocalIndexInTriangle<numberOfVerticesPerTriangle ; ++vertexLocalIndexInTriangle)
		{
			unsigned vertexIndex = gridData.triangleConnectivity(triangleIndex,vertexLocalIndexInTriangle);
			triangle.addVertex(grid2D.vertices[vertexIndex]);
		}
		grid2D.triangles.push_back(triangle);
	}
	return ;
}

void GridBuilder::buildQuadranglesOn2DGrid(const GridData& gridData, Grid2D& grid2D)
{
	const unsigned numberOfVerticesPerQuadrangle = 4;
	const unsigned numberOfQuadrangles = gridData.quadrangleConnectivity.rows();
	grid2D.quadrangles.reserve(numberOfQuadrangles);
	for(unsigned quadrangleIndex=0 ; quadrangleIndex<numberOfQuadrangles ; ++quadrangleIndex)
	{
		Quadrangle quadrangle;
		for(unsigned vertexLocalIndexInQuadrangle=0 ; vertexLocalIndexInQuadrangle<numberOfVerticesPerQuadrangle ; ++vertexLocalIndexInQuadrangle)
		{
			unsigned vertexIndex = gridData.quadrangleConnectivity(quadrangleIndex,vertexLocalIndexInQuadrangle);
			quadrangle.addVertex(grid2D.vertices[vertexIndex]);
		}
		grid2D.quadrangles.push_back(quadrangle);
	}
	return ;
}

void GridBuilder::assignElementsPointers(Grid2D& grid2D)
{
	for(Triangle& triangle: grid2D.triangles)
		grid2D.elements.push_back((Element*) &triangle);
	for(Quadrangle& quadrangle: grid2D.quadrangles)
		grid2D.elements.push_back((Element*) &quadrangle);
	return ;
}
