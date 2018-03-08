#ifndef GRID2D_HPP
#define GRID2D_HPP

#include <vector>
#include <GeometricEntity/Quadrangle.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <Grid/Grid.hpp>
#include <Grid/GridData.hpp>

struct Grid2D: public Grid
{
	public:
		Grid2D(const GridData& gridData);

		std::vector<Quadrangle> quadrangles;
		std::vector<Triangle> triangles;
	private:
		void buildTrianglesOn2DGrid(const GridData& gridData);
		void buildQuadranglesOn2DGrid(const GridData& gridData);
		void assignElementsPointers(void);
};

#endif
