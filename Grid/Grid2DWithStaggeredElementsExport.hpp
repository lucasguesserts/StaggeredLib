#ifndef GRID_2D_WITH_STAGGERED_ELEMENTS_HPP
#define GRID_2D_WITH_STAGGERED_ELEMENTS_HPP

#include <Utils/String.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <CgnsInterface/CgnsCreator/CgnsCreator2D.hpp>

class Grid2DWithStaggeredElementsExport
{
	public:
		static void cgns(const std::string& fileName, const Grid2DWithStaggeredElements& grid);
	private:
		static void exportCoordinates(const Grid2DWithStaggeredElements& grid, GridDataShared gridData);
		static void exportStaggeredTriangles(const Grid2DWithStaggeredElements& grid, GridDataShared gridData);
		static void exportStaggeredQuadrangles(const Grid2DWithStaggeredElements& grid, GridDataShared gridData);
		static void exportLines(const Grid2DWithStaggeredElements& grid, GridDataShared gridData);
};

#endif