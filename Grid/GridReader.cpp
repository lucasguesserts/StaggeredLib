#include <Grid/GridReader.hpp>
#include <cgnslib.h>

#define NAME_LENGTH 200

const std::string GridReader::projectGridDirectory = GRID_DIRECTORY;

GridData GridReader::CGNS(const std::string fileName)
{
	int file, base;
	char baseName[NAME_LENGTH], zoneName[NAME_LENGTH];
	int numberOfBases, cellDimension, physicalDimension;
	//int numberOfZones;
	//cgsize_t size[3];
	//ZoneType_t zoneType;
	// file
	cg_open(fileName.c_str(),CG_MODE_READ,&file);
	// base
	cg_nbases(file, &numberOfBases);
	if(numberOfBases!=1) cg_error_exit();
	else base = 1;
	cg_base_read(file,base,baseName,&cellDimension,&physicalDimension);
	if(cellDimension=!2) cg_error_exit();
	if(physicalDimension!=2) cg_error_exit();
	//// zone
	//cg_nzones(file, base, &numberOfZones);
	//if(numberOfZones!=1) cg_erro_exit();
	//else zone = 1;
	//cg_zone_type(file,base,zone,&zoneType);
	//if(zoneType!=Unstructured) cg_error_exit();
	//cg_zone_read(file,base,zone,zoneName,size);
		//// 'size' for 2D unstructured: NVertex, NCell2D, NBoundVertex

	// GridData
	GridData gridData;
	gridData.dimension = 2;
	return gridData;
}
