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
		Grid2D(const std::string& fileName);

		std::vector<Quadrangle> quadrangles;
		std::vector<Triangle> triangles;
		std::vector<Line> lines;
	private:
		void buildLinesOn2DGrid(void);
		void buildTrianglesOn2DGrid(void);
		void buildQuadranglesOn2DGrid(void);

		void buildLinesOn2DGrid_2(const GridData_2& gridData);
		void buildTrianglesOn2DGrid_2(const GridData_2& gridData);
		void buildQuadranglesOn2DGrid_2(const GridData_2& gridData);

		void assignElementsPointers(void);
};

#endif
