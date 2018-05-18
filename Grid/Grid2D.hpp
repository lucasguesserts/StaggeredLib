#ifndef GRID2D_HPP
#define GRID2D_HPP

#include <vector>
#include <GeometricEntity/Quadrangle.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/Line.hpp>
#include <Grid/Grid.hpp>

struct Grid2D: public Grid
{
	public:
		Grid2D(const std::string& fileName);

		std::vector<Quadrangle> quadrangles;
		std::vector<Triangle> triangles;
		std::vector<Line> lines;
	private:
		void buildLinesOn2DGrid(void);
		void buildTrianglesOn2DGrid(void);
		void buildQuadranglesOn2DGrid(void);

		void assignElementsPointers(void);
};

#endif