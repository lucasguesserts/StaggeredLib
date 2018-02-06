#include <Grid/VertexCollection.hpp>

VertexCollection::VertexCollection(const unsigned numberOfVertices)
{
	vertices.reserve(numberOfVertices);
}

void VertexCollection::addVertex(Vertex& vertex)
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

std::vector<Vertex *>& VertexCollection::getVertices(void)
{
	return this->vertices;
}
