#include <CGNSFile/CGNSFile.hpp>
#include <boost/format.hpp>
#include <stdexcept>

#define NAME_LENGTH 200
#define FUNCTION_ERROR_MESSAGE std::string("CGNSFile::") + std::string(__FUNCTION__) + std::string(": ")

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
	error = cgns::cg_open(cgnsFileName.c_str(),CGNS_ENUMV(CG_MODE_MODIFY),&(this->fileIndex));
	if(error) throw std::invalid_argument(FUNCTION_ERROR_MESSAGE + std::string("File not found: ") + cgnsFileName);
	return;
}

void CGNSFile::openBase(void)
{
	int error, numberOfBases;
	char baseName[NAME_LENGTH];
	error = cgns::cg_nbases(this->fileIndex, &numberOfBases);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of bases.");
		if(numberOfBases!=1) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Invalid number of bases: 1!=" + std::to_string(numberOfBases));
		else this->baseIndex = 1;
	error = cgns::cg_base_read(this->fileIndex,this->baseIndex,baseName,&(this->cellDimension),&(this->physicalDimension));
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read base.");
	return;
}

void CGNSFile::openZone(void)
{
	int error, numberOfZones;
	char zoneName[NAME_LENGTH];
	cgns::ZoneType_t zoneType;
	cgns::cgsize_t size[3];
	error = cgns::cg_nzones(this->fileIndex,this->baseIndex, &numberOfZones);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of zones.");
		if(numberOfZones!=1) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Invalid number of zones: 1!=" + std::to_string(numberOfZones));
		else this->zoneIndex = 1;
	error = cgns::cg_zone_type(this->fileIndex,this->baseIndex,this->zoneIndex,&zoneType);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read zone type.");
		if(zoneType!=CGNS_ENUMV(cgns::Unstructured)) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Only unstructured zones are supported.");
	error = cgns::cg_zone_read(this->fileIndex,this->baseIndex,this->zoneIndex,zoneName,size);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read zone.");
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
	error = cgns::cg_coord_read(this->fileIndex, this->baseIndex, this->zoneIndex, coordinateName.c_str(), CGNS_ENUMV(cgns::RealDouble), &range_min, &range_max, coordinates.data());
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read coordinate." + coordinateName);
	return coordinates;
}

void CGNSFile::verifyNumberOfGrids(void)
{
	int error;
	int numberOfGrids;
	error = cgns::cg_ngrids(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfGrids);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of grids.");
	if(numberOfGrids!=1) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "The CGNS file has more than one grid.");
	return;
}

void CGNSFile::verifyNumberOfCoordinates(void)
{
	int error;
	int numberOfCoordinates;
	error = cgns::cg_ncoords(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfCoordinates);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of coordinates.");
	if(numberOfCoordinates!=this->physicalDimension) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Number of coordinates incompatible with physical dimension.");
	return;
}

void CGNSFile::readNumberOfSections(void)
{
	int error;
	error = cgns::cg_nsections(this->fileIndex,this->baseIndex,this->zoneIndex,&(this->numberOfSections));
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of sections.");
	this->checkNumberOfSections();
	return;
}

void CGNSFile::checkNumberOfSections(void)
{
	if(this->numberOfSections < 1)
		throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Invalid number of sections: 1!=" + std::to_string(numberOfSections));
	return;
}

std::vector<cgns::cgsize_t> CGNSFile::readElementConnectivity(const int sectionIndex)
{
	int error;
	cgns::cgsize_t sizeOfElementConnectivityDataArray = readSizeOfElementConnectivityDataArray(sectionIndex);
	std::vector<cgns::cgsize_t> elementConnectivity(sizeOfElementConnectivityDataArray);
	error = cgns::cg_elements_read(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,elementConnectivity.data(),nullptr);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read elements connectivity section " + std::to_string(sectionIndex));
	return elementConnectivity;
}

cgns::cgsize_t CGNSFile::readSizeOfElementConnectivityDataArray(const int sectionIndex)
{
	int error;
	cgns::cgsize_t sizeOfElementConnectivityDataArray;
	error = cgns::cg_ElementDataSize(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,&sizeOfElementConnectivityDataArray);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read element data size in section " + std::to_string(sectionIndex));
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
	char elementSectionName[NAME_LENGTH];
	cgns::ElementType_t elementType;
	cgns::cgsize_t firstElementIndex, lastElementIndex;
	int numberOfBoundaries, parentFlag;
	int error = cg_section_read(this->fileIndex,this->baseIndex,this->zoneIndex,sectionIndex,elementSectionName,&elementType,&firstElementIndex,&lastElementIndex,&numberOfBoundaries,&parentFlag);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read section " + std::to_string(sectionIndex));
		--firstElementIndex; --lastElementIndex; // The CGNS enumeration starts at 1, Here it starts at 0.
	return std::tuple<cgns::cgsize_t,cgns::cgsize_t,cgns::ElementType_t>(firstElementIndex,lastElementIndex,elementType);
}

void CGNSFile::writeSteadyScalarField(const std::string& solutionName, const std::string& scalarFieldName, const Eigen::VectorXd& scalarField)
{
	int error;
	const double* scalarFieldRawData = &scalarField[0];
	int solutionIndex, fieldIndex;
	error = cgns::cg_sol_write(this->fileIndex,this->baseIndex,this->zoneIndex, solutionName.c_str(), CGNS_ENUMV(cgns::CellCenter), &solutionIndex);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write solution " + solutionName);
	error = cgns::cg_field_write(this->fileIndex,this->baseIndex,this->zoneIndex, solutionIndex, CGNS_ENUMV(cgns::RealDouble), scalarFieldName.c_str(), scalarFieldRawData, &fieldIndex);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write field " + scalarFieldName);
	return;
}

Eigen::VectorXd CGNSFile::readSteadyScalarField(const std::string& solutionName, const std::string& scalarFieldName)
{
	const int solutionIndex = this->getSolutionIndex(solutionName);
	this->verifyGridLocationOfSolution(solutionIndex);
	const cgns::cgsize_t solutionSize = this->readSolutionSize(solutionIndex);
	Eigen::VectorXd steadySolution(solutionSize);
	cgns::cgsize_t rangeMin=1, rangeMax=solutionSize;
	int error = cgns::cg_field_read(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,scalarFieldName.c_str(),CGNS_ENUMV(cgns::RealDouble),&rangeMin,&rangeMax,&steadySolution[0]);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read field " + scalarFieldName);
	return steadySolution;
}

int CGNSFile::getSolutionIndex(const std::string solutionName)
{
	int error;
	int numberOfSolutions;
	error = cgns::cg_nsols(this->fileIndex,this->baseIndex,this->zoneIndex,&numberOfSolutions);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of solutions.");
	int solutionIndex;
	for(solutionIndex=1 ; solutionIndex<=numberOfSolutions ; ++solutionIndex)
	{
		char readSolutionName[NAME_LENGTH];
		cgns::GridLocation_t gridLocation;
		error = cg_sol_info(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,readSolutionName,&gridLocation);
			if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read solution " + std::to_string(solutionIndex) + " information.");
		if(solutionName.compare(readSolutionName)==0) break;
	}
	return solutionIndex;
}

void CGNSFile::verifyGridLocationOfSolution(const int solutionIndex)
{
	char solutionName[NAME_LENGTH];
	cgns::GridLocation_t gridLocation;
	int error = cgns::cg_sol_info(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,solutionName,&gridLocation);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read solution " + std::to_string(solutionIndex) + " information.");
	if(gridLocation!=CGNS_ENUMV(cgns::CellCenter)) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Only CellCenter grid location supported.");
	return;
}

cgns::cgsize_t CGNSFile::readSolutionSize(const int solutionIndex)
{
	int solutionDimension;
	cgns::cgsize_t solutionSize;
	int error = cgns::cg_sol_size(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,&solutionDimension,&solutionSize);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read solution " + std::to_string(solutionIndex) + " size.");
	return solutionSize;
}

void CGNSFile::writeTransientScalarField(const std::string& scalarFieldName, const unsigned timeStep, const Eigen::VectorXd scalarField)
{
	int error;
	std::string solutionName = this->getSolutionName(scalarFieldName, timeStep);
	int solutionIndex, fieldIndex;
	error = cgns::cg_sol_write(this->fileIndex,this->baseIndex,this->zoneIndex,solutionName.c_str(),CGNS_ENUMV(cgns::CellCenter),&solutionIndex);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write solution " + solutionName);
	error = cgns::cg_field_write(this->fileIndex,this->baseIndex,this->zoneIndex,solutionIndex,CGNS_ENUMV(cgns::RealDouble),scalarFieldName.c_str(),&scalarField[0], &fieldIndex);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write field " + scalarFieldName);
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
	int error = cgns::cg_biter_write(this->fileIndex,this->baseIndex,timeIterativeBaseName.c_str(),numberOfTimeSteps);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write time iterative base " + timeIterativeBaseName);
	return;
}

void CGNSFile::writeTimeInstantsInIterativeBase(const std::string& timeIterativeBaseName, const Eigen::VectorXd& timeInstants)
{
	int error;
	constexpr int arraySize = 1;
	const std::string arrayName = "Time_Instants";
	cgns::cgsize_t timeArrayDimension[arraySize] = { static_cast<cgns::cgsize_t>(timeInstants.size()) };
	error = cgns::cg_goto(this->fileIndex,this->baseIndex,timeIterativeBaseName.c_str(),0,"end");
		if(error) throw std::invalid_argument(FUNCTION_ERROR_MESSAGE + "Could not go to " + timeIterativeBaseName);
	error = cgns::cg_array_write(arrayName.c_str(),CGNS_ENUMV(cgns::RealDouble),arraySize,timeArrayDimension,&timeInstants[0]);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write array " + arrayName);
	return;
}

void CGNSFile::writeTimeIterativeZone(const std::string& scalarFieldName, const std::string& timeIterativeZoneName, const Eigen::VectorXd& timeInstants)
{
	int error;
	const unsigned solutionNameSize = this->getSolutionName(scalarFieldName,0).size();
	const unsigned numberOfTimeSteps = timeInstants.size();
	error = cgns::cg_ziter_write(this->fileIndex,this->baseIndex,this->zoneIndex,timeIterativeZoneName.c_str());
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write time iterative zone " + timeIterativeZoneName);
	error = cgns::cg_goto(this->fileIndex,this->baseIndex,"Zone_t",this->zoneIndex,timeIterativeZoneName.c_str(),0,"end");
		if(error) throw std::invalid_argument(FUNCTION_ERROR_MESSAGE + "Could not go to " + timeIterativeZoneName);
	cgns::cgsize_t solutionNameDimension[2];
		solutionNameDimension[0] = static_cast<cgns::cgsize_t>(solutionNameSize);
		solutionNameDimension[1] = static_cast<cgns::cgsize_t>(numberOfTimeSteps);
	std::string allSolutionNames = this->getSolutionNamesForTimeIterativeZone(scalarFieldName,numberOfTimeSteps);
	const std::string solutionPointersArrayName = "SolutionPointers";
	error = cgns::cg_array_write(solutionPointersArrayName.c_str(),CGNS_ENUMV(cgns::Character),2,solutionNameDimension,allSolutionNames.c_str());
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write array " + solutionPointersArrayName);
	return;
}

std::string CGNSFile::getSolutionName(const std::string& scalarFieldName, const unsigned timeStep)
{
	// time width in name equals to 6. Defined two lines below
	std::string solutionName = (boost::format("%s_%06d") % scalarFieldName % (timeStep+1)).str(); /* Paraview start at one */
	return solutionName;
}


std::string CGNSFile::getSolutionNamesForTimeIterativeZone(const std::string& scalarFieldName, const unsigned numberOfTimeSteps)
{
	const unsigned solutionNameSize = this->getSolutionName(scalarFieldName,0).size();
	std::string allSolutionNames;
	allSolutionNames.reserve(solutionNameSize);
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
		allSolutionNames += this->getSolutionName(scalarFieldName, timeStep);
	return allSolutionNames;
}

void CGNSFile::writeSimulationType(void)
{
	int error = cgns::cg_simulation_type_write(this->fileIndex,this->baseIndex,CGNS_ENUMV(cgns::TimeAccurate));
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not write time simulation type.");
	return;
}

Eigen::VectorXd CGNSFile::readTransientScalarField(const std::string& scalarFieldName, const unsigned timeStep)
{
	std::string solutionName = this->getSolutionName(scalarFieldName,timeStep);
	return readSteadyScalarField(solutionName,scalarFieldName);
}

unsigned CGNSFile::readNumberOfTimeSteps(void)
{
	char timeIterativeBaseName[NAME_LENGTH];
	int numberOfTimeSteps;
	int error = cgns::cg_biter_read(this->fileIndex,this->baseIndex,timeIterativeBaseName,&numberOfTimeSteps);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read time iterative base information.");
	return static_cast<unsigned>(numberOfTimeSteps);
}

Eigen::VectorXd CGNSFile::readAllTimeInstants(void)
{
	int arrayIndex = this->getArrayIndex();
	this->verifyArrayInformation(arrayIndex);
	return this->readTimeArrayData(arrayIndex);
}

int CGNSFile::getArrayIndex(void)
{
	int error;
	error = cgns::cg_goto(this->fileIndex,this->baseIndex,"BaseIterativeData_t",1,"end");
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not go to time iterative base.");
	int numberOfArrays;
	error = cgns::cg_narrays(&numberOfArrays);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of array.");
	if(numberOfArrays!=1) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + std::string("Invalid number of arrays: 1!=") + std::to_string(numberOfArrays));
	int arrayIndex = 1;
	error = cgns::cg_goto(this->fileIndex,this->baseIndex,"end");
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not go to file base.");
	return arrayIndex;
}

void CGNSFile::verifyArrayInformation(const int arrayIndex)
{
	int error;
	error = cgns::cg_goto(this->fileIndex,this->baseIndex,"BaseIterativeData_t",1,"end");
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not go to time iterative base.");
	char arrayName[NAME_LENGTH];
	int dataDimension;
	cgns::cgsize_t dataDimensionVector[12];
	cgns::DataType_t arrayDataType;
	error = cgns::cg_array_info(arrayIndex,arrayName,&arrayDataType,&dataDimension,dataDimensionVector);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read array " + std::to_string(arrayIndex) + " information.");
	if(arrayDataType!=CGNS_ENUMV(cgns::RealDouble)) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Array '" + std::string(arrayName) + "' data type must be RealDouble.");
	constexpr int arraySize = 1; if(dataDimension!=arraySize) cgns::cg_error_exit();
	if(dataDimensionVector[0]!=this->readNumberOfTimeSteps()) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + std::string("Inconsistent Array '") + std::string(arrayName) + std::string("'dimension and number of time steps") + std::string("\n") + std::to_string(dataDimensionVector[0]) + std::string(" != ") + std::to_string(this->readNumberOfTimeSteps()));
	return;
}

Eigen::VectorXd CGNSFile::readTimeArrayData(const int arrayIndex)
{
	unsigned numberOfTimeInstants = this->readNumberOfTimeSteps();
	Eigen::VectorXd allTimeInstants(numberOfTimeInstants);
	int error = cgns::cg_array_read(arrayIndex, &allTimeInstants[0]);
		if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read array " + std::to_string(arrayIndex) + " data.");
	return allTimeInstants;
}


int CGNSFile::readNumberOfBoundaries()
{
	int error, numberOfBoundaries;
	error = cgns::cg_nbocos(this->fileIndex,this->baseIndex,this->zoneIndex, &numberOfBoundaries);
	if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read number of boundaries.");
	return numberOfBoundaries;
}


BoundaryDefinition CGNSFile::readBoundaryDefinition(const unsigned boundaryIndex)
{
	int error;
	BoundaryDefinition boundary;
	boundary.index = boundaryIndex;
	error = cgns::cg_boco_info(this->fileIndex,this->baseIndex,this->zoneIndex,
	                           boundaryIndex,
	                           boundary.name,
	                           &boundary.type,
	                           &boundary.pointSetType,
	                           &boundary.numberOfElements,
	                           &boundary.normalIndex,
	                           &boundary.normalListSize,
	                           &boundary.dataType,
	                           &boundary.numberOfDataSets);
	if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read boundary " + std::to_string(boundary.index) + " information.");
	if(boundary.type!=CGNS_ENUMV(cgns::BCDirichlet)) throw std::runtime_error("Boundary condition type is " + std::string(cgns::BCTypeName[boundary.type]) + ", but it must be 'BCDirichlet'.");
	if(boundary.pointSetType!=CGNS_ENUMV(cgns::PointList)) throw std::runtime_error("Point set type is " + std::string(cgns::PointSetTypeName[boundary.pointSetType]) + ", but it must be 'PointList'.");
	boundary.elementsIndexList = this->readBoundaryElementList(boundary.index, boundary.numberOfElements);
	return boundary;
}

std::vector<unsigned> CGNSFile::readBoundaryElementList(int boundaryIndex, cgns::cgsize_t numberOfElements)
{
	int error;
	std::vector<cgns::cgsize_t> elementsCGNSIndex(numberOfElements);
	error = cgns::cg_boco_read(this->fileIndex,this->baseIndex,this->zoneIndex,boundaryIndex,&elementsCGNSIndex[0],nullptr);
	if(error) throw std::runtime_error(FUNCTION_ERROR_MESSAGE + "Could not read boundary " + std::to_string(boundaryIndex) + " elements.");
	return CGNSFile::transformCGNSIndices(elementsCGNSIndex);
}

std::vector<unsigned> CGNSFile::transformCGNSIndices(const std::vector<cgns::cgsize_t>& elementsCGNSIndex)
{
	std::vector<unsigned> elementsIndices (elementsCGNSIndex.cbegin(),elementsCGNSIndex.cend());
	for(auto& entry: elementsIndices)
		--entry;
	return elementsIndices;
}

std::vector<BoundaryDefinition> CGNSFile::readBoundaries(void)
{
	int numberOfBoundaries = this->readNumberOfBoundaries();
	std::vector<BoundaryDefinition> boundaries;
	for(int boundaryIndex=1 ; boundaryIndex<=numberOfBoundaries ; ++boundaryIndex)
		boundaries.emplace_back(std::move( readBoundaryDefinition(boundaryIndex) ));
	return boundaries;
}