#include <GeometricEntity/Entity.hpp>

Entity::Entity(const unsigned handle)
	: handle(handle) {}

void Entity::setHandle(const unsigned handle)
{
	this->handle = handle;
	return;
}

unsigned Entity::getHandle(void) const
{
	return this->handle;
}
