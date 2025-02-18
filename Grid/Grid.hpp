#ifndef GRID_HPP
#define GRID_HPP

#include <vector>
#include <memory>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <CgnsInterface/CgnsReader/CgnsReader2D.hpp>

const std::string gridDirectory = GRID_DIRECTORY;

struct Grid
{
	public:
		Grid(const std::string& fileName);

		unsigned dimension;
		std::vector<Vertex> vertices;
		std::vector<Element*> elements;
		std::unique_ptr<CgnsReader2D> cgnsReader;
	private:
		void readCgnsFile(const std::string& fileName);
		void buildVertices(void);
};

#endif
