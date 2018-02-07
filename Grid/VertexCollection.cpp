#include <Grid/VertexCollection.hpp>

VertexCollection::VertexCollection(const unsigned numberOfVertices)
{
	vertices.reserve(numberOfVertices);
}

void VertexCollection::addVertex(const Vertex& vertex)
{
	vertices.push_back(&vertex);
}

unsigned VertexCollection::getNumberOfVertices(void) const
{
	return this->vertices.size();
}

Vertex VertexCollection::getVertex(const unsigned vertexLocalHandle) const
{
	return *(this->vertices[vertexLocalHandle]);
}

std::vector<const Vertex *> VertexCollection::getVertices(void) const
{
	return this->vertices;
}
