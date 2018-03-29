#ifndef STAGGERED_ELEMENT_DEFINITION_HPP
#define STAGGERED_ELEMENT_DEFINITION_HPP

#include <array>

struct StaggeredElementDefinition
{
	enum Type {Triangle, Quadrangle};
	std::array<unsigned,2> vertices;
	std::array<unsigned,2> elements;
	StaggeredElementDefinition::Type type;

	StaggeredElementDefinition(const unsigned firstVertexIndex, const unsigned secondVertexIndex, const unsigned elementIndex);
	void addElement(const unsigned elementIndex);
};

bool operator==(const StaggeredElementDefinition& lhs, const StaggeredElementDefinition& rhs);

#endif
