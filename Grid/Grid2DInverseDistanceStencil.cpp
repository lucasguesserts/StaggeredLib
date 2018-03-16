#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <array>

Grid2DInverseDistanceStencil::Grid2DInverseDistanceStencil(GridData& gridData)
	: Grid2DWithStaggeredElements(gridData)
{}

ScalarStencil Grid2DInverseDistanceStencil::computeScalarStencil(Vertex& vertex)
{
	ScalarStencil vertexScalarStencil;
	for(Element* element: this->verticesNeighborElements[vertex.getIndex()])
		vertexScalarStencil[element->getIndex()] = 1 / (vertex - element->getCentroid()).norm();
	Grid2DInverseDistanceStencil::normalizeScalarStencil(vertexScalarStencil);
	return vertexScalarStencil;
}

void Grid2DInverseDistanceStencil::normalizeScalarStencil(ScalarStencil& scalarStencil)
{
	double sum = 0.0;
	for(auto& keyValuePair: scalarStencil)
		sum += keyValuePair.second;
	for(auto& keyValuePair: scalarStencil)
		keyValuePair.second *= (1/sum);
	return;
}

std::vector<ScalarStencil> Grid2DInverseDistanceStencil::computeScalarStencilOnVertices(void)
{
	std::vector<ScalarStencil> scalarStencilOnVertices;
	scalarStencilOnVertices.reserve(this->vertices.size());
	for(Vertex& vertex: this->vertices)
		scalarStencilOnVertices.emplace_back(this->computeScalarStencil(vertex));
	return scalarStencilOnVertices;
}

ScalarStencil Grid2DInverseDistanceStencil::computeScalarStencilOnElement(Element* element)
{
	return {{element->getIndex(),1.0}};
}

VectorStencil Grid2DInverseDistanceStencil::computeVectorStencilOnQuadrangle(StaggeredQuadrangle& staggeredQuadrangle, std::vector<ScalarStencil>& scalarStencilOnVertices)
{
	std::array<VectorStencil,4> vectorStencilOnQuadrangleFaces;
	vectorStencilOnQuadrangleFaces[0] = Grid2DInverseDistanceStencil::computeVectorStencil(*(staggeredQuadrangle.vertices[0]), staggeredQuadrangle.elements[0]->getCentroid(), scalarStencilOnVertices[staggeredQuadrangle.vertices[0]->getIndex()], Grid2DInverseDistanceStencil::computeScalarStencilOnElement(staggeredQuadrangle.elements[0]));
	vectorStencilOnQuadrangleFaces[1] = Grid2DInverseDistanceStencil::computeVectorStencil(staggeredQuadrangle.elements[0]->getCentroid(), *(staggeredQuadrangle.vertices[1]), Grid2DInverseDistanceStencil::computeScalarStencilOnElement(staggeredQuadrangle.elements[0]), scalarStencilOnVertices[staggeredQuadrangle.vertices[1]->getIndex()]);
	vectorStencilOnQuadrangleFaces[2] = Grid2DInverseDistanceStencil::computeVectorStencil(*(staggeredQuadrangle.vertices[1]), staggeredQuadrangle.elements[1]->getCentroid(), scalarStencilOnVertices[staggeredQuadrangle.vertices[1]->getIndex()], Grid2DInverseDistanceStencil::computeScalarStencilOnElement(staggeredQuadrangle.elements[1]));
	vectorStencilOnQuadrangleFaces[3] = Grid2DInverseDistanceStencil::computeVectorStencil(staggeredQuadrangle.elements[1]->getCentroid(), *(staggeredQuadrangle.vertices[0]), Grid2DInverseDistanceStencil::computeScalarStencilOnElement(staggeredQuadrangle.elements[1]), scalarStencilOnVertices[staggeredQuadrangle.vertices[0]->getIndex()]);
	return (1/staggeredQuadrangle.getVolume()) * (vectorStencilOnQuadrangleFaces[0] + vectorStencilOnQuadrangleFaces[1] + vectorStencilOnQuadrangleFaces[2] + vectorStencilOnQuadrangleFaces[3]);
}

VectorStencil Grid2DInverseDistanceStencil::computeVectorStencil(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint, const ScalarStencil& firstScalarStencil, const ScalarStencil& secondScalarStencil)
{
	Eigen::Vector3d areaVector = Grid2DInverseDistanceStencil::computeAreaVector(firstPoint,secondPoint);
	ScalarStencil averageScalarStencil = computeAverageScalarStencil(firstScalarStencil,secondScalarStencil);
	return averageScalarStencil*areaVector;
}

Eigen::Vector3d Grid2DInverseDistanceStencil::computeAreaVector(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint)
{
	Eigen::Vector3d areaVector = secondPoint - firstPoint;
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}

ScalarStencil Grid2DInverseDistanceStencil::computeAverageScalarStencil(const ScalarStencil& firstScalarStencil, const ScalarStencil& secondScalarStencil)
{
	return 0.5 * (firstScalarStencil + secondScalarStencil);
}
