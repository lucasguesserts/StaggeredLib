#include <Grid/CGNSFile.hpp>

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

std::vector<double> CGNSFile::readCoordinate(const std::string& coordinateName)
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

void CGNSFile::readNumberOfSections(void)
{
	int error;
	error = cgns::cg_nsections(this->fileIndex,this->baseIndex,this->zoneIndex,&(this->numberOfSections)); CHKERRQ(error);
	if(this->numberOfSections < 1) cgns::cg_error_exit();
	return;
}


std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> CGNSFile::readSection(const int sectionIndex)
{
	int error;
	char elementSectionName[NAME_LENGTH];
	cgns::ElementType_t elementType;
	cgns::cgsize_t firstElementIndex, lastElementIndex;
	int numberOfBoundaries, parentFlag;
	error = cg_section_read(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,elementSectionName,&elementType,&firstElementIndex,&lastElementIndex,&numberOfBoundaries,&parentFlag); CHKERRQ(error);
		--firstElementIndex; --lastElementIndex; // The CGNS enumeration starts at 1, Here it starts at 0.
	return std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t>(firstElementIndex,lastElementIndex,elementType);
}

cgns::cgsize_t CGNSFile::readSizeOfElementConnectivityDataArray(const int sectionIndex)
{
	int error;
	cgns::cgsize_t sizeOfElementConnectivityDataArray;
	error = cgns::cg_ElementDataSize(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,&sizeOfElementConnectivityDataArray); CHKERRQ(error);
	return sizeOfElementConnectivityDataArray;
}

std::vector<cgns::cgsize_t> CGNSFile::readElementConnectivity(const int sectionIndex)
{
	int error;
	cgns::cgsize_t sizeOfElementConnectivityDataArray = readSizeOfElementConnectivityDataArray(sectionIndex);
	std::vector<cgns::cgsize_t> elementConnectivity(sizeOfElementConnectivityDataArray);
	error = cgns::cg_elements_read(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,elementConnectivity.data(),NULL); CHKERRQ(error);
	return elementConnectivity;
}

unsigned CGNSFile::getNumberOfElementsOfType(const cgns::ElementType_t elementType)
{
	unsigned numberOfElements = 0;
	for(int sectionIndex=1 ; sectionIndex<=this->numberOfSections ; ++sectionIndex)
	{
		std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> sectionData = this->readSection(sectionIndex);
		if(std::get<2>(sectionData)==elementType) numberOfElements += (std::get<1>(sectionData) - std::get<0>(sectionData) + 1);
	}
	return numberOfElements;
}

std::vector< ElementDefinition<4> > CGNSFile::readQuadrangleElementsDefinition(void)
{
	CGNSFile::readNumberOfSections();
	unsigned numberOfQuadrangles = this->getNumberOfElementsOfType(cgns::QUAD_4);
	std::vector< ElementDefinition<4> > quadrangle(numberOfQuadrangles);
	for(int sectionIndex=1 ; sectionIndex<=this->numberOfSections ; ++sectionIndex)
	{
		std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> sectionData = this->readSection(sectionIndex);
		if(std::get<2>(sectionData)==cgns::QUAD_4)
		{
			const cgns::cgsize_t firstElementIndex = std::get<0>(sectionData);
			const cgns::cgsize_t lastElementIndex = std::get<1>(sectionData);
			std::vector<cgns::cgsize_t> elementConnectivity = this->readElementConnectivity(sectionIndex);
			for(unsigned quadrangleCount=0 ; quadrangleCount<numberOfQuadrangles ; ++quadrangleCount)
			{
				quadrangle[quadrangleCount].index = firstElementIndex + quadrangleCount;
				for(unsigned vertexIndex=0 ; vertexIndex<4 ; ++vertexIndex)
					quadrangle[quadrangleCount].connectivity(vertexIndex) = elementConnectivity[4*quadrangleCount+vertexIndex] - 1;
			}
		}
	}
	return quadrangle;
}
