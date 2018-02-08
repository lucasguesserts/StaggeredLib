#include <Grid/GridReader.hpp>
#include <cgnslib.h>
#include <Eigen/Core>
#include <iostream>
using std::cout;
using std::endl;

#define NAME_LENGTH 200
#define CHKERRQ(err) if((err)) cg_error_exit()

const std::string GridReader::projectGridDirectory = GRID_DIRECTORY;

GridData GridReader::CGNS(const std::string fileName)
{
	int error;
	// GridData
	GridData gridData;
	gridData.dimension = 2;
	// file
	int file;
		error = cg_open(fileName.c_str(),CG_MODE_READ,&file); CHKERRQ(error);
	// base
	char baseName[NAME_LENGTH];
	int base, numberOfBases, cellDimension, physicalDimension;
		error = cg_nbases(file, &numberOfBases); CHKERRQ(error);
		if(numberOfBases!=1) cg_error_exit();
		else base = 1;
		error = cg_base_read(file,base,baseName,&cellDimension,&physicalDimension); CHKERRQ(error);
		if(cellDimension!=2) cg_error_exit();
		if(physicalDimension!=2) cg_error_exit();
	// zone
	char zoneName[NAME_LENGTH];
	int zone, numberOfZones;
	ZoneType_t zoneType;
	cgsize_t size[3];
		error = cg_nzones(file, base, &numberOfZones); CHKERRQ(error);
		if(numberOfZones!=1) cg_error_exit();
		else zone = 1;
		error = cg_zone_type(file,base,zone,&zoneType); CHKERRQ(error);
		if(zoneType!=Unstructured) cg_error_exit();
		error = cg_zone_read(file,base,zone,zoneName,size); CHKERRQ(error);
			// 'size' for 2D unstructured: NVertex, NCell2D, NBoundVertex
	// grid
	int numberOfGrids;
		error = cg_ngrids(file,base,zone,&numberOfGrids); CHKERRQ(error);
		if(numberOfGrids!=1) cg_error_exit();
	// coordinates
	int numberOfCoordinates;
	cgsize_t range_min=1, range_max=size[0];
	double *coordinatesX, *coordinatesY;
		error = cg_ncoords(file,base,zone,&numberOfCoordinates); CHKERRQ(error);
		if(numberOfCoordinates!=2) cg_error_exit();
		coordinatesX = (double *) malloc(size[0]*sizeof(double));
		coordinatesY = (double *) malloc(size[0]*sizeof(double));
		error = cg_coord_read(file, base, zone, "CoordinateX", CGNS_ENUMV(RealDouble), &range_min, &range_max, coordinatesX); CHKERRQ(error);
		error = cg_coord_read(file, base, zone, "CoordinateY", CGNS_ENUMV(RealDouble), &range_min, &range_max, coordinatesY); CHKERRQ(error);
	gridData.coordinates.resize(size[0],Eigen::NoChange);
	for(unsigned vertexIndex=0 ; vertexIndex<size[0] ; ++vertexIndex)
	{
		gridData.coordinates(vertexIndex,0) = coordinatesX[vertexIndex];
		gridData.coordinates(vertexIndex,1) = coordinatesY[vertexIndex];
		gridData.coordinates(vertexIndex,2) = 0.0;
	}
	// Element connectivity - section
	int numberOfSections;
	error = cg_nsections(file,base,zone,&numberOfSections); CHKERRQ(error);
	if(numberOfSections<1) cg_error_exit();
	for(int section=1 ; section<=numberOfSections ; ++section)
	{
		char elementSectionName[NAME_LENGTH];
		ElementType_t elementType;
		cgsize_t sectionStart, sectionEnd, elementDataSize;
		cgsize_t *elementConnectivity;
		int numberOfBoundaries, parentFlag;
		error = cg_section_read(file,base,zone,section,elementSectionName,&elementType,&sectionStart,&sectionEnd,&numberOfBoundaries,&parentFlag); CHKERRQ(error);
		if(elementType==CGNS_ENUMV(QUAD_4))
		{
			error = cg_ElementDataSize(file,base,zone,section,&elementDataSize); CHKERRQ(error);
			elementConnectivity = new cgsize_t[elementDataSize];
			error = cg_elements_read(file,base,zone,section,elementConnectivity,NULL); CHKERRQ(error);
			if(elementDataSize%4 != 0) cg_error_exit();
			const unsigned numberOfQuadrangles = elementDataSize / 4;
			gridData.quadrangleConnectivity.resize(numberOfQuadrangles,Eigen::NoChange);
			for(unsigned quadrangle=0 ; quadrangle<numberOfQuadrangles ; ++quadrangle)
				for(unsigned vertexIndex=0 ; vertexIndex<4 ; ++vertexIndex)
					gridData.quadrangleConnectivity(quadrangle,vertexIndex) = elementConnectivity[4*quadrangle+vertexIndex] - 1;
		}
		if(elementType==CGNS_ENUMV(TRI_3))
		{
			error = cg_ElementDataSize(file,base,zone,section,&elementDataSize); CHKERRQ(error);
			elementConnectivity = new cgsize_t[elementDataSize];
			error = cg_elements_read(file,base,zone,section,elementConnectivity,NULL); CHKERRQ(error);
			if(elementDataSize%3 != 0) cg_error_exit();
			const unsigned numberOfTriangles = elementDataSize / 3;
			gridData.triangleConnectivity.resize(numberOfTriangles,Eigen::NoChange);
			for(unsigned triangle=0 ; triangle<numberOfTriangles ; ++triangle)
				for(unsigned vertexIndex=0 ; vertexIndex<3 ; ++vertexIndex)
					gridData.triangleConnectivity(triangle,vertexIndex) = elementConnectivity[3*triangle+vertexIndex] - 1;
		}
	}
	return gridData;
}
