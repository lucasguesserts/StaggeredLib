#include <Grid/StaggeredElementDefinition.hpp>

StaggeredElementDefinition::StaggeredElementDefinition(const unsigned firstVertexIndex, const unsigned secondVertexIndex, const unsigned elementIndex)
{
	this->vertices[0] = firstVertexIndex;
	this->vertices[1] = secondVertexIndex;
	this->elements[0] = elementIndex;
	this->type = StaggeredElementDefinition::Type::Triangle;
	return;
}

void StaggeredElementDefinition::addElement(const unsigned elementIndex)
{
	this->elements[1] = elementIndex;
	this->type = StaggeredElementDefinition::Type::Quadrangle;
	return;
}

bool operator==(const StaggeredElementDefinition& lhs, const StaggeredElementDefinition& rhs)
{
	return
		( (lhs.vertices[0]==rhs.vertices[0]) && (lhs.vertices[1]==rhs.vertices[1]) )
		||
		( (lhs.vertices[0]==rhs.vertices[1]) && (lhs.vertices[1]==rhs.vertices[0]) )
		;
}
