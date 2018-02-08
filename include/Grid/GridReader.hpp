#ifndef GRID_READER_HPP
#define GRID_READER_HPP

#include <Grid/GridData.hpp>
#include <string>


class GridReader
{
	public:
		static GridData CGNS(const std::string fileName);
		static const std::string projectGridDirectory;
};

#endif
