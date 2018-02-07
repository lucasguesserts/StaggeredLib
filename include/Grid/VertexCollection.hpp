#ifndef VERTEX_COLLECTION_HPP
#define VERTEX_COLLECTION_HPP

#include <Grid/Vertex.hpp>
#include <vector>

class VertexCollection
{
	public:
		explicit VertexCollection(const unsigned numberOfVertices = 0);
		void addVertex(const Vertex& vertex);
		unsigned getNumberOfVertices(void) const;
		Vertex getVertex(const unsigned vertexLocalHandle) const;
		std::vector<const Vertex *> getVertices(void) const;

	private:
		std::vector<const Vertex *> vertices;
};

#endif
