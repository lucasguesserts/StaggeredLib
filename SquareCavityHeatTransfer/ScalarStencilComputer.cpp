#include <SquareCavityHeatTransfer/ScalarStencilComputer.hpp>
#include <algorithm>

ScalarStencil ScalarStencilComputer::inverseDistance(const Vertex& vertex,const std::vector<Element*>& vertexNeighborElements)
{
	ScalarStencil vertexScalarStencil;
	for(Element* element: vertexNeighborElements)
		vertexScalarStencil[element->getIndex()] = 1 / (vertex - element->getCentroid()).norm();
	ScalarStencilComputer::normalizeScalarStencil(vertexScalarStencil);
	return vertexScalarStencil;
}

void ScalarStencilComputer::normalizeScalarStencil(ScalarStencil& scalarStencil)
{
	double sum = 0.0;
	for(auto& keyValuePair: scalarStencil)
		sum += keyValuePair.second;
	for(auto& keyValuePair: scalarStencil)
		keyValuePair.second *= (1/sum);
	return;
}

std::vector<ScalarStencil> ScalarStencilComputer::inverseDistance(const Grid2DVerticesWithNeighborElements& grid)
{
	std::vector<ScalarStencil> scalarStencilOnVertices;
	scalarStencilOnVertices.reserve(grid.vertices.size());
	for(const Vertex& vertex: grid.vertices)
		scalarStencilOnVertices.emplace_back( ScalarStencilComputer::inverseDistance(vertex,grid.verticesNeighborElements[vertex.getIndex()]));
	return scalarStencilOnVertices;
}

std::vector<ScalarStencil> ScalarStencilComputer::elements(const Grid2DVerticesWithNeighborElements& grid)
{
	constexpr double weightValue = 1.0;
	std::vector<ScalarStencil> scalarStencilOnElements;
	scalarStencilOnElements.reserve(grid.elements.size());
	for(Element* element: grid.elements)
	{
		ScalarStencil elementScalarStencil = { {element->getIndex(), weightValue} };
		scalarStencilOnElements.emplace_back(elementScalarStencil);
	}
	return scalarStencilOnElements;
}

VectorStencil ScalarStencilComputer::vectorStencil(StaggeredQuadrangle& staggeredQuadrangle, std::vector<ScalarStencil>& scalarStencilOnVertices, std::vector<ScalarStencil>& scalarStencilOnElements)
{
	std::array<VectorStencil,4> vectorStencilOnQuadrangleFaces;
	vectorStencilOnQuadrangleFaces[0] = ScalarStencilComputer::computeVectorStencil(*(staggeredQuadrangle.vertices[0]), staggeredQuadrangle.elements[0]->getCentroid(), scalarStencilOnVertices[staggeredQuadrangle.vertices[0]->getIndex()], scalarStencilOnElements[staggeredQuadrangle.elements[0]->getIndex()]);
	vectorStencilOnQuadrangleFaces[1] = ScalarStencilComputer::computeVectorStencil(staggeredQuadrangle.elements[0]->getCentroid(), *(staggeredQuadrangle.vertices[1]), scalarStencilOnElements[staggeredQuadrangle.elements[0]->getIndex()], scalarStencilOnVertices[staggeredQuadrangle.vertices[1]->getIndex()]);
	vectorStencilOnQuadrangleFaces[2] = ScalarStencilComputer::computeVectorStencil(*(staggeredQuadrangle.vertices[1]), staggeredQuadrangle.elements[1]->getCentroid(), scalarStencilOnVertices[staggeredQuadrangle.vertices[1]->getIndex()], scalarStencilOnElements[staggeredQuadrangle.elements[1]->getIndex()]);
	vectorStencilOnQuadrangleFaces[3] = ScalarStencilComputer::computeVectorStencil(staggeredQuadrangle.elements[1]->getCentroid(), *(staggeredQuadrangle.vertices[0]), scalarStencilOnElements[staggeredQuadrangle.elements[1]->getIndex()], scalarStencilOnVertices[staggeredQuadrangle.vertices[0]->getIndex()]);
	return (1/staggeredQuadrangle.getVolume()) * (vectorStencilOnQuadrangleFaces[0] + vectorStencilOnQuadrangleFaces[1] + vectorStencilOnQuadrangleFaces[2] + vectorStencilOnQuadrangleFaces[3]);
}

VectorStencil ScalarStencilComputer::vectorStencil(StaggeredTriangle& staggeredTriangle, std::vector<ScalarStencil>& scalarStencilOnVertices, std::vector<ScalarStencil>& scalarStencilOnElements)
{
	std::array<VectorStencil,3> vectorStencilOnTriangleFaces;
	vectorStencilOnTriangleFaces[0] = ScalarStencilComputer::computeVectorStencil(*(staggeredTriangle.vertices[0]), staggeredTriangle.element->getCentroid(), scalarStencilOnVertices[staggeredTriangle.vertices[0]->getIndex()], scalarStencilOnElements[staggeredTriangle.element->getIndex()]);
	vectorStencilOnTriangleFaces[1] = ScalarStencilComputer::computeVectorStencil(staggeredTriangle.element->getCentroid(), *(staggeredTriangle.vertices[1]), scalarStencilOnElements[staggeredTriangle.element->getIndex()], scalarStencilOnVertices[staggeredTriangle.vertices[1]->getIndex()]);
	vectorStencilOnTriangleFaces[2] = ScalarStencilComputer::computeVectorStencil(*(staggeredTriangle.vertices[1]), *(staggeredTriangle.vertices[0]), scalarStencilOnVertices[staggeredTriangle.vertices[1]->getIndex()], scalarStencilOnVertices[staggeredTriangle.vertices[0]->getIndex()]);
	return (1/staggeredTriangle.getVolume()) * (vectorStencilOnTriangleFaces[0] + vectorStencilOnTriangleFaces[1] + vectorStencilOnTriangleFaces[2]);
}

VectorStencil ScalarStencilComputer::computeVectorStencil(Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint, ScalarStencil& firstScalarStencil, ScalarStencil& secondScalarStencil)
{
	Eigen::Vector3d areaVector = ScalarStencilComputer::computeAreaVector(firstPoint,secondPoint);
	ScalarStencil averageScalarStencil = computeAverageScalarStencil(firstScalarStencil,secondScalarStencil);
	return averageScalarStencil*areaVector;
}

Eigen::Vector3d ScalarStencilComputer::computeAreaVector(Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint)
{
	Eigen::Vector3d areaVector = secondPoint - firstPoint;
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}

ScalarStencil ScalarStencilComputer::computeAverageScalarStencil(ScalarStencil& firstScalarStencil, ScalarStencil& secondScalarStencil)
{
	return 0.5 * (firstScalarStencil + secondScalarStencil);
}
