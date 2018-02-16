#ifndef GRID_ENTITIES_ENTITY_HPP
#define GRID_ENTITIES_ENTITY_HPP

class Entity {
public:
	explicit Entity(const unsigned index = 0);
	void setIndex(const unsigned index);
	unsigned getIndex(void) const;
private:
	unsigned index;
};

#endif
