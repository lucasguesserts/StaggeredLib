#ifndef GRID2D_HPP
#define GRID2D_HPP

#include <vector>
#include <GeometricEntity/Quadrangle.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <GeometricEntity/Line.hpp>
#include <Grid/Grid.hpp>
#include <Grid/GridData_2.hpp>

struct Grid2D: public Grid
{
	public:
		Grid2D(const GridData_2& gridData);

		std::vector<Quadrangle> quadrangles;
		std::vector<Triangle> triangles;
		std::vector<Line> lines;
	private:
		void buildLinesOn2DGrid(const GridData_2& gridData);
		void buildTrianglesOn2DGrid(const GridData_2& gridData);
		void buildQuadranglesOn2DGrid(const GridData_2& gridData);
		void assignElementsPointers(void);
};

#endif
