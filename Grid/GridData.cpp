#include <Grid/GridData.hpp>
#include <cgnslib.h>

const std::string GridData::projectGridDirectory = GRID_DIRECTORY;

#define NAME_LENGTH 200
#define CHKERRQ(err) if((err)) cg_error_exit()

void GridData::openFile(const std::string cgnsFileName)
{
	int error;
	error = cg_open(cgnsFileName.c_str(),CGNS_ENUMV(CG_MODE_READ),&(this->fileIndex)); CHKERRQ(error);
	return;
}

void GridData::openBase(void)
{
	int error;
	char baseName[NAME_LENGTH];
	int base, numberOfBases, cellDimension, physicalDimension;
	this->dimension = 2;
	error = cg_nbases(this->fileIndex, &numberOfBases); CHKERRQ(error);
		if(numberOfBases!=1) cg_error_exit();
		else this->baseIndex = 1;
	error = cg_base_read(this->fileIndex,this->baseIndex,baseName,&cellDimension,&physicalDimension); CHKERRQ(error);
		if(cellDimension!=this->dimension) cg_error_exit();
		if(physicalDimension!=this->dimension) cg_error_exit();
	return;
}

void GridData::openZone(void)
{
	int error;
	char zoneName[NAME_LENGTH];
	int zone, numberOfZones;
	ZoneType_t zoneType;
	cgsize_t size[3];
	error = cg_nzones(this->fileIndex,this->baseIndex, &numberOfZones); CHKERRQ(error);
		if(numberOfZones!=1) cg_error_exit();
		else this->zoneIndex = 1;
	error = cg_zone_type(this->fileIndex,this->baseIndex,this->zoneIndex,&zoneType); CHKERRQ(error);
		if(zoneType!=CGNS_ENUMV(Unstructured)) cg_error_exit();
	error = cg_zone_read(this->fileIndex,this->baseIndex,this->zoneIndex,zoneName,size); CHKERRQ(error);
	this->numberOfVertices = size[0];
	this->numberOfElements = size[1];
	return;
}

void GridData::verifyNumberOfGrids(void)
{
	int error;
	int numberOfGrids;
	error = cg_ngrids(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfGrids); CHKERRQ(error);
	if(numberOfGrids!=1) cg_error_exit();
	return;
}

GridData::GridData(const std::string cgnsFileName)
{
	int error;
	this->dimension = 2;

	this->openFile(cgnsFileName);
	this->openBase();
	this->openZone();
	this->verifyNumberOfGrids();
	// coordinates
	int numberOfCoordinates;
	cgsize_t range_min=1, range_max=this->numberOfVertices;
	double *coordinatesX, *coordinatesY;
		error = cg_ncoords(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfCoordinates); CHKERRQ(error);
		if(numberOfCoordinates!=2) cg_error_exit();
		coordinatesX = new double[this->numberOfVertices];
		coordinatesY = new double[this->numberOfVertices];
		error = cg_coord_read(this->fileIndex, this->baseIndex, this->zoneIndex, "CoordinateX", CGNS_ENUMV(RealDouble), &range_min, &range_max, coordinatesX); CHKERRQ(error);
		error = cg_coord_read(this->fileIndex, this->baseIndex, this->zoneIndex, "CoordinateY", CGNS_ENUMV(RealDouble), &range_min, &range_max, coordinatesY); CHKERRQ(error);
	this->coordinates.resize(this->numberOfVertices,Eigen::NoChange);
	for(unsigned vertexIndex=0 ; vertexIndex<this->numberOfVertices ; ++vertexIndex)
	{
		this->coordinates(vertexIndex,0) = coordinatesX[vertexIndex];
		this->coordinates(vertexIndex,1) = coordinatesY[vertexIndex];
		this->coordinates(vertexIndex,2) = 0.0;
	}
	delete coordinatesX;
	delete coordinatesY;
	// Element connectivity - section
	int numberOfSections;
	error = cg_nsections(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfSections); CHKERRQ(error);
	if(numberOfSections<1) cg_error_exit();
	for(int section=1 ; section<=numberOfSections ; ++section)
	{
		char elementSectionName[NAME_LENGTH];
		ElementType_t elementType;
		cgsize_t sectionStart, sectionEnd, elementDataSize;
		cgsize_t *elementConnectivity;
		int numberOfBoundaries, parentFlag;
		error = cg_section_read(this->fileIndex,this->baseIndex,this->zoneIndex,section,elementSectionName,&elementType,&sectionStart,&sectionEnd,&numberOfBoundaries,&parentFlag); CHKERRQ(error);
		if(elementType==CGNS_ENUMV(QUAD_4))
		{
			error = cg_ElementDataSize(this->fileIndex,this->baseIndex,this->zoneIndex,section,&elementDataSize); CHKERRQ(error);
			elementConnectivity = new cgsize_t[elementDataSize];
			error = cg_elements_read(this->fileIndex,this->baseIndex,this->zoneIndex,section,elementConnectivity,NULL); CHKERRQ(error);
			if(elementDataSize%4 != 0) cg_error_exit();
			const unsigned numberOfQuadrangles = elementDataSize / 4;
			this->quadrangleConnectivity.resize(numberOfQuadrangles,Eigen::NoChange);
			for(unsigned quadrangle=0 ; quadrangle<numberOfQuadrangles ; ++quadrangle)
				for(unsigned vertexIndex=0 ; vertexIndex<4 ; ++vertexIndex)
					this->quadrangleConnectivity(quadrangle,vertexIndex) = elementConnectivity[4*quadrangle+vertexIndex] - 1;
		}
		if(elementType==CGNS_ENUMV(TRI_3))
		{
			error = cg_ElementDataSize(this->fileIndex,this->baseIndex,this->zoneIndex,section,&elementDataSize); CHKERRQ(error);
			elementConnectivity = new cgsize_t[elementDataSize];
			error = cg_elements_read(this->fileIndex,this->baseIndex,this->zoneIndex,section,elementConnectivity,NULL); CHKERRQ(error);
			if(elementDataSize%3 != 0) cg_error_exit();
			const unsigned numberOfTriangles = elementDataSize / 3;
			this->triangleConnectivity.resize(numberOfTriangles,Eigen::NoChange);
			for(unsigned triangle=0 ; triangle<numberOfTriangles ; ++triangle)
				for(unsigned vertexIndex=0 ; vertexIndex<3 ; ++vertexIndex)
					this->triangleConnectivity(triangle,vertexIndex) = elementConnectivity[3*triangle+vertexIndex] - 1;
		}
	}
	return;
}
