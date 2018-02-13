#ifndef GRID_BUILDER_HPP
#define GRID_BUILDER_HPP

#include <Grid/GridData.hpp>
#include <Grid/Grid2D.hpp>

class GridBuilder: public Grid
{
	public:
		static Grid2D build2D(const GridData gridData);
	private:
		static bool gridIs2D(const GridData& gridData);
		static void buildVerticesOn2DGrid(const GridData& gridData, Grid2D& grid2D);
		static void buildTrianglesOn2DGrid(const GridData& gridData, Grid2D& grid2D);
		static void buildQuadranglesOn2DGrid(const GridData& gridData, Grid2D& grid2D);
		static void assignElementsPointers(Grid2D& grid2D);
};

#endif
