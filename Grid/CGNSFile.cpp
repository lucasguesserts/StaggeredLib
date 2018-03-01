#include <Grid/CGNSFile.hpp>
namespace cgns
{
	#include <cgnslib.h>
}

#define NAME_LENGTH 200
#define CHKERRQ(err) if((err)) cgns::cg_error_exit()

CGNSFile::CGNSFile(const std::string cgnsFileName)
{
	this->openFile(cgnsFileName);
	this->openBase();
	this->openZone();
	return;
}

void CGNSFile::openFile(const std::string cgnsFileName)
{
	int error;
	error = cgns::cg_open(cgnsFileName.c_str(),CGNS_ENUMV(CG_MODE_MODIFY),&(this->fileIndex)); CHKERRQ(error);
	return;
}

void CGNSFile::openBase(void)
{
	int error, numberOfBases;
	char baseName[NAME_LENGTH];
	error = cgns::cg_nbases(this->fileIndex, &numberOfBases); CHKERRQ(error);
		if(numberOfBases!=1) cgns::cg_error_exit();
		else this->baseIndex = 1;
	error = cgns::cg_base_read(this->fileIndex,this->baseIndex,baseName,&(this->cellDimension),&(this->physicalDimension)); CHKERRQ(error);
	return;
}

void CGNSFile::openZone(void)
{
	int error, numberOfZones;
	char zoneName[NAME_LENGTH];
	cgns::ZoneType_t zoneType;
	cgns::cgsize_t size[3];
	error = cgns::cg_nzones(this->fileIndex,this->baseIndex, &numberOfZones); CHKERRQ(error);
		if(numberOfZones!=1) cgns::cg_error_exit();
		else this->zoneIndex = 1;
	error = cgns::cg_zone_type(this->fileIndex,this->baseIndex,this->zoneIndex,&zoneType); CHKERRQ(error);
		if(zoneType!=CGNS_ENUMV(cgns::Unstructured)) cgns::cg_error_exit();
	error = cgns::cg_zone_read(this->fileIndex,this->baseIndex,this->zoneIndex,zoneName,size); CHKERRQ(error);
	this->numberOfVertices = size[0];
	this->numberOfElements = size[1];
	return;
}

std::vector<double> CGNSFile::readCoordinates(const std::string& coordinateName)
{
	this->verifyNumberOfGrids();
	this->verifyNumberOfCoordinates();
	int error;
	cgns::cgsize_t range_min=1, range_max=this->numberOfVertices;
	std::vector<double> coordinates(numberOfVertices);
	error = cgns::cg_coord_read(this->fileIndex, this->baseIndex, this->zoneIndex, coordinateName.c_str(), CGNS_ENUMV(cgns::RealDouble), &range_min, &range_max, coordinates.data()); CHKERRQ(error);
	return coordinates;
}

void CGNSFile::verifyNumberOfGrids(void)
{
	int error;
	int numberOfGrids;
	error = cgns::cg_ngrids(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfGrids); CHKERRQ(error);
	if(numberOfGrids!=1) cgns::cg_error_exit();
	return;
}

void CGNSFile::verifyNumberOfCoordinates(void)
{
	int error;
	int numberOfCoordinates;
	error = cgns::cg_ncoords(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfCoordinates); CHKERRQ(error);
	if(numberOfCoordinates!=this->physicalDimension) cgns::cg_error_exit();
	return;
}
