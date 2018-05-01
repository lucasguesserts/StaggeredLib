#include <GeometricEntity/StaggeredTriangle.hpp>

StaggeredTriangle::StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
	: StaggeredElement(index, vertex_0, element, vertex_1)
{}