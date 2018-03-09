#include <SquareCavityHeatTransfer/ScalarStencilComputer.hpp>
#include <utility>

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
	VectorStencil vectorStencilOnFacet;
	VectorStencil firstScalarStencil = ScalarStencilComputer::computeVectorStencil(*(staggeredQuadrangle.vertices[0]), staggeredQuadrangle.elements[0]->getCentroid(), scalarStencilOnVertices[staggeredQuadrangle.vertices[0]->getIndex()], scalarStencilOnElements[staggeredQuadrangle.elements[0]->getIndex()]);
	VectorStencil correctVectorStencil = {{ { 0, {0.75,-0.75,0.0} }, { 1, {-0.75,0.75,0.0} } }};
	return correctVectorStencil;
}

VectorStencil ScalarStencilComputer::computeVectorStencil(Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint, ScalarStencil& firstScalarStencil, ScalarStencil& secondScalarStencil)
{
	Eigen::Vector3d vectorPerpendicularToAreaVector = secondPoint - firstPoint;
	Eigen::Vector3d areaVector = {vectorPerpendicularToAreaVector[1], -vectorPerpendicularToAreaVector[0], vectorPerpendicularToAreaVector[2]}; //rotate
	ScalarStencil averageScalarStencil = 0.5 * (firstScalarStencil + secondScalarStencil);
	return averageScalarStencil*areaVector;
}
