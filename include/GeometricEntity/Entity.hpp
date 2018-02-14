#ifndef GRID_ENTITIES_ENTITY_HPP
#define GRID_ENTITIES_ENTITY_HPP

class Entity {
public:
	explicit Entity(const unsigned handle = 0);
	void setHandle(const unsigned handle);
	unsigned getHandle(void) const;
private:
	unsigned handle;
};

#endif
