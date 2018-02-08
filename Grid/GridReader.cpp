#include <Grid/GridReader.hpp>
#include <cgnslib.h>

#define NAME_LENGTH 200

const std::string GridReader::projectGridDirectory = GRID_DIRECTORY;

GridData GridReader::CGNS(const std::string fileName)
{
	// file
	int file;
		cg_open(fileName.c_str(),CG_MODE_READ,&file);
	// base
	char baseName[NAME_LENGTH];
	int base, numberOfBases, cellDimension, physicalDimension;
		cg_nbases(file, &numberOfBases);
		if(numberOfBases!=1) cg_error_exit();
		else base = 1;
		cg_base_read(file,base,baseName,&cellDimension,&physicalDimension);
		if(cellDimension!=2) cg_error_exit();
		if(physicalDimension!=2) cg_error_exit();
	// zone
	char zoneName[NAME_LENGTH];
	int zone, numberOfZones;
	ZoneType_t zoneType;
	cgsize_t size[3];
		cg_nzones(file, base, &numberOfZones);
		if(numberOfZones!=1) cg_error_exit();
		else zone = 1;
		cg_zone_type(file,base,zone,&zoneType);
		if(zoneType!=Unstructured) cg_error_exit();
		cg_zone_read(file,base,zone,zoneName,size);
			// 'size' for 2D unstructured: NVertex, NCell2D, NBoundVertex
	// grid
	int numberOfGrids;
		cg_ngrids(file,base,zone,&numberOfGrids);
		if(numberOfGrids!=1) cg_error_exit();
	// coordinates
	int numberOfCoordinates;
	cgsize_t range_min=1, range_max=size[0];
	double *coordinatesX, *coordinatesY;
		cg_ncoords(file,base,zone,&numberOfCoordinates);
		if(numberOfCoordinates!=2) cg_error_exit();
		coordinatesX = (double *) malloc(size[0]*sizeof(double));
		coordinatesY = (double *) malloc(size[0]*sizeof(double));
		cg_coord_read(file, base, zone, "CoordinateX", CGNS_ENUMV(RealDouble), &range_min, &range_max, coordinatesX);
		cg_coord_read(file, base, zone, "CoordinateY", CGNS_ENUMV(RealDouble), &range_min, &range_max, coordinatesY);
	// GridData
	GridData gridData;
	gridData.dimension = 2;
	return gridData;
}
