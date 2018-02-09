#ifndef GRID_BUILDER_HPP
#define GRID_BUILDER_HPP

#include <Grid/GridData.hpp>
#include <Grid/Grid2D.hpp>

class GridBuilder: public Grid
{
	public:
		static Grid2D build2D(const GridData gridData);
};

#endif
