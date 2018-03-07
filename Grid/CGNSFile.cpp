#include <Grid/CGNSFile.hpp>
#include <boost/format.hpp>

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
	this->checkNumberOfSections();
	return;
}

void CGNSFile::checkNumberOfSections(void)
{
	if(this->numberOfSections < 1) cgns::cg_error_exit();
	return;
}

std::vector<cgns::cgsize_t> CGNSFile::readElementConnectivity(const int sectionIndex)
{
	int error;
	cgns::cgsize_t sizeOfElementConnectivityDataArray = readSizeOfElementConnectivityDataArray(sectionIndex);
	std::vector<cgns::cgsize_t> elementConnectivity(sizeOfElementConnectivityDataArray);
	error = cgns::cg_elements_read(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,elementConnectivity.data(),NULL); CHKERRQ(error);
	return elementConnectivity;
}

cgns::cgsize_t CGNSFile::readSizeOfElementConnectivityDataArray(const int sectionIndex)
{
	int error;
	cgns::cgsize_t sizeOfElementConnectivityDataArray;
	error = cgns::cg_ElementDataSize(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,&sizeOfElementConnectivityDataArray); CHKERRQ(error);
	return sizeOfElementConnectivityDataArray;
}

unsigned CGNSFile::getNumberOfElementsOfType(const cgns::ElementType_t elementType)
{
	unsigned numberOfElements = 0;
	this->readNumberOfSections();
	for(int sectionIndex=1 ; sectionIndex<=this->numberOfSections ; ++sectionIndex)
	{
		std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t> sectionData = this->readSection(sectionIndex);
		if(std::get<2>(sectionData)==elementType) numberOfElements += (std::get<1>(sectionData) - std::get<0>(sectionData) + 1);
	}
	return numberOfElements;
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

void CGNSFile::writeSteadyScalarField(const std::string& solutionName, const std::string& scalarFieldName, const Eigen::VectorXd& scalarField)
{
	int error;
	const double* scalarFieldRawData = &scalarField[0];
	int solutionIndex, fieldIndex;
	error = cgns::cg_sol_write(this->fileIndex,this->baseIndex,this->zoneIndex, solutionName.c_str(), CGNS_ENUMV(cgns::CellCenter), &solutionIndex); CHKERRQ(error);
	error = cgns::cg_field_write(this->fileIndex,this->baseIndex,this->zoneIndex, solutionIndex, CGNS_ENUMV(cgns::RealDouble), scalarFieldName.c_str(), scalarFieldRawData, &fieldIndex); CHKERRQ(error);
	return;
}

Eigen::VectorXd CGNSFile::readSteadyScalarField(const std::string& solutionName, const std::string& scalarFieldName)
{
	const int solutionIndex = this->getSolutionIndex(solutionName);
	this->verifyGridLocationOfSolution(solutionIndex);
	const cgns::cgsize_t solutionSize = this->readSolutionSize(solutionIndex);
	Eigen::VectorXd steadySolution;
	steadySolution.resize(solutionSize);
	cgns::cgsize_t rangeMin=1, rangeMax=solutionSize;
	int error;
	error = cgns::cg_field_read(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,scalarFieldName.c_str(),CGNS_ENUMV(cgns::RealDouble),&rangeMin,&rangeMax,&steadySolution[0]); CHKERRQ(error);
	return steadySolution;
}

int CGNSFile::getSolutionIndex(const std::string solutionName)
{
	int error;
	int numberOfSolutions;
	error = cgns::cg_nsols(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfSolutions); CHKERRQ(error);
	int solutionIndex;
	for(solutionIndex=1 ; solutionIndex<=numberOfSolutions ; ++solutionIndex)
	{
		char readSolutionName[NAME_LENGTH];
		cgns::GridLocation_t gridLocation;
		error = cg_sol_info(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,readSolutionName,&gridLocation); CHKERRQ(error);
		if(solutionName.compare(readSolutionName)==0) break;
	}
	return solutionIndex;
}

void CGNSFile::verifyGridLocationOfSolution(const int solutionIndex)
{
	int error;
	char solutionName[NAME_LENGTH];
	cgns::GridLocation_t gridLocation;
	error = cgns::cg_sol_info(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,solutionName,&gridLocation); CHKERRQ(error);
	if(gridLocation!=CGNS_ENUMV(cgns::CellCenter)) cgns::cg_error_exit();
	return;
}

cgns::cgsize_t CGNSFile::readSolutionSize(const int solutionIndex)
{
	int error;
	int solutionDimension;
	cgns::cgsize_t solutionSize;
	error = cgns::cg_sol_size(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,&solutionDimension,&solutionSize);
	return solutionSize;
}

void CGNSFile::writeTransientScalarField(const std::string& scalarFieldName, const unsigned timeStep, const Eigen::VectorXd scalarField)
{
	int error;
	std::string solutionName = this->getSolutionName(scalarFieldName, timeStep);
	int solutionIndex, fieldIndex;
	error = cgns::cg_sol_write(this->fileIndex,this->baseIndex,this->zoneIndex,solutionName.c_str(),CGNS_ENUMV(cgns::CellCenter),&solutionIndex); CHKERRQ(error);
	error = cgns::cg_field_write(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,CGNS_ENUMV(cgns::RealDouble),scalarFieldName.c_str(),&scalarField[0], &fieldIndex); CHKERRQ(error);
	return;
}

void CGNSFile::writeTransientInformation(const std::string& scalarFieldName, const Eigen::VectorXd& timeInstants)
{
	const std::string timeIterativeBaseName = scalarFieldName + "_base";
	const std::string timeIterativeZoneName = scalarFieldName + "_zone";
	this->writeTimeIterativeBase(timeIterativeBaseName,timeInstants.size());
	this->writeTimeInstantsInIterativeBase(timeIterativeBaseName,timeInstants);
	this->writeTimeIterativeZone(scalarFieldName,timeIterativeZoneName,timeInstants);
	this->writeSimulationType();
	return;
}

void CGNSFile::writeTimeIterativeBase(const std::string& timeIterativeBaseName, const int numberOfTimeSteps)
{
	int error;
	error = cgns::cg_biter_write(this->fileIndex,this->baseIndex,timeIterativeBaseName.c_str(),numberOfTimeSteps); CHKERRQ(error);
	return;
}

void CGNSFile::writeTimeInstantsInIterativeBase(const std::string& timeIterativeBaseName, const Eigen::VectorXd& timeInstants)
{
	int error;
	cgns::cgsize_t timeArrayDimension[1] = { static_cast<cgns::cgsize_t>(timeInstants.size()) };
	error = cgns::cg_goto(this->fileIndex,this->baseIndex,timeIterativeBaseName.c_str(),0,"end"); CHKERRQ(error);
	error = cgns::cg_array_write("Time_Instants",CGNS_ENUMV(cgns::RealDouble),1,timeArrayDimension,&timeInstants[0]); CHKERRQ(error);
	return;
}

void CGNSFile::writeTimeIterativeZone(const std::string& scalarFieldName, const std::string& timeIterativeZoneName, const Eigen::VectorXd& timeInstants)
{
	int error;
	const unsigned numberOfTimeSteps = timeInstants.size();
	const unsigned timeWidthInSolutionName = 5;
	error = cgns::cg_ziter_write(this->fileIndex,this->baseIndex,this->zoneIndex,timeIterativeZoneName.c_str()); CHKERRQ(error);
	error = cgns::cg_goto(this->fileIndex,this->baseIndex,"Zone_t",this->zoneIndex,timeIterativeZoneName.c_str(),0,"end"); CHKERRQ(error);
	cgns::cgsize_t solutionNameDimension[2];
	solutionNameDimension[0] = scalarFieldName.size() + 1 + timeWidthInSolutionName;
	solutionNameDimension[1] = static_cast<cgns::cgsize_t>(numberOfTimeSteps);
	//const unsigned solutionNameSize = scalarFieldName.size() + timeWidthInSolutionName;
	std::string solutionNames;
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		solutionNames += this->getSolutionName(scalarFieldName, timeStep);
	error = cgns::cg_array_write("SolutionPointers",CGNS_ENUMV(cgns::Character),2,solutionNameDimension,solutionNames.c_str()); CHKERRQ(error);
	return;
}

std::string CGNSFile::getSolutionName(const std::string& scalarFieldName, const unsigned timeStep)
{
	constexpr unsigned timeWidthInSolutionName = 5;
	const unsigned solutionNameSize = scalarFieldName.size() + timeWidthInSolutionName;
	char* solutionName = new char[solutionNameSize];
	sprintf(solutionName, "%s_%05d", scalarFieldName.c_str(), timeStep+1); /* Paraview start at one */
	return std::string(solutionName);
}

void CGNSFile::writeSimulationType(void)
{
	int error = cgns::cg_simulation_type_write(this->fileIndex,this->baseIndex,CGNS_ENUMV(cgns::TimeAccurate)); CHKERRQ(error);
	return;
}
