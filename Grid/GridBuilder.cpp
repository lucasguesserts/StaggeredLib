#include <Grid/GridBuilder.hpp>
#include <Grid/Vertex.hpp>

Grid2D GridBuilder::build2D(const GridData gridData)
{
	Grid2D grid2D;
	// Dimension
	grid2D.dimension = gridData.dimension;
	// Vertices
	const unsigned numberOfVertices = gridData.coordinates.rows();
	grid2D.vertices.reserve(numberOfVertices);
	for(unsigned vertexIndex=0 ; vertexIndex<numberOfVertices ; ++vertexIndex)
	{
		const double x = gridData.coordinates(vertexIndex,0);
		const double y = gridData.coordinates(vertexIndex,1);
		const double z = gridData.coordinates(vertexIndex,2);
		grid2D.vertices.push_back(Vertex(x, y, z, vertexIndex));
	}
	// Triangles
	const unsigned numberOfTriangles = gridData.triangleConnectivity.rows();
	grid2D.triangles.reserve(numberOfTriangles);
	for(unsigned triangleIndex=0 ; triangleIndex<numberOfTriangles ; ++triangleIndex)
	{
		Triangle triangle;
		for(unsigned vertexIndexInTriangle=0 ; vertexIndexInTriangle<3 ; ++vertexIndexInTriangle)
		{
			unsigned vertexIndex = gridData.triangleConnectivity(triangleIndex,vertexIndexInTriangle);
			triangle.vertices.push_back(&(grid2D.vertices[vertexIndex]));
		}
		grid2D.triangles.push_back(triangle);
	}
	// Quadrangles
	const unsigned numberOfQuadrangles = gridData.quadrangleConnectivity.rows();
	grid2D.quadrangles.reserve(numberOfQuadrangles);
	for(unsigned quadrangleIndex=0 ; quadrangleIndex<numberOfQuadrangles ; ++quadrangleIndex)
	{
		Quadrangle quadrangle;
		for(unsigned vertexIndexInQuadrangle=0 ; vertexIndexInQuadrangle<4 ; ++vertexIndexInQuadrangle)
		{
			unsigned vertexIndex = gridData.quadrangleConnectivity(quadrangleIndex,vertexIndexInQuadrangle);
			quadrangle.vertices.push_back(&(grid2D.vertices[vertexIndex]));
		}
		grid2D.quadrangles.push_back(quadrangle);
	}
	// Elements
	for(Triangle& triangle: grid2D.triangles)
		grid2D.elements.push_back((Element*) &triangle);
	for(Quadrangle& quadrangle: grid2D.quadrangles)
		grid2D.elements.push_back((Element*) &quadrangle);
	return grid2D;
}
