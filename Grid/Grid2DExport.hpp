#ifndef GRID_2D_EXPORT_HPP
#define GRID_2D_EXPORT_HPP

#include <Utils/String.hpp>
#include <Grid/Grid2D.hpp>
#include <CgnsInterface/CgnsCreator/CgnsCreator2D.hpp>

#include <iostream>
#include <fstream>
#include <Utils/String.hpp>
#define debug std::cout << __FILE__ << ": " << __LINE__ << std::endl;

class Grid2DExport
{
	public:
		static void cgns(const std::string& fileName, const Grid2D& grid);
		static void csv(const std::string& fileName, Grid2D& grid);
			static void csvAppendTimeSolution(const std::string& fileName, const double timeInstant, const std::vector<double>& solution);
	private:
		static void exportCoordinates(const Grid2D& grid, GridDataShared gridData);
		static void exportTriangles(const Grid2D& grid, GridDataShared gridData);
		static void exportQuadrangles(const Grid2D& grid, GridDataShared gridData);
		static void exportRegion(const Grid2D& grid, GridDataShared gridData);
};

#endif