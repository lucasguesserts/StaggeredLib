#include <GeometricEntity/Entity.hpp>

Entity::Entity(const unsigned index)
	: index(index) {}

void Entity::setIndex(const unsigned index)
{
	this->index = index;
	return;
}

unsigned Entity::getIndex(void) const
{
	return this->index;
}
