#include <SquareCavityHeatTransfer/GridSave.hpp>
#include <cgnslib.h>
//#include <cstddef>

#define NAME_LENGTH 200
#define CHKERRQ(err) if((err)) cg_error_exit()

void GridSave::savePermanentScalarFieldToCGNS(const std::string solutionName, const std::string scalarFieldName, const Eigen::VectorXd& scalarField, const std::string cgnsFileName)
{
	int error;
	// check if it is a cgns file
	int fileType;
	error = cg_is_cgns(cgnsFileName.c_str(),&fileType); CHKERRQ(error);
	// file
	int file;
		error = cg_open(cgnsFileName.c_str(),CGNS_ENUMV(CG_MODE_MODIFY),&file); CHKERRQ(error);
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
		if(zoneType!=CGNS_ENUMV(Unstructured)) cg_error_exit();
		error = cg_zone_read(file,base,zone,zoneName,size); CHKERRQ(error);
			// 'size' for 2D unstructured: NVertex, NCell2D, NBoundVertex
	// get raw poiter to memory
	const double* scalarFieldRawData = &scalarField[0];
	//// Write solution to file
	int solutionIndex, field;
	error = cg_sol_write(file, base, zone, solutionName.c_str(), CGNS_ENUMV(CellCenter), &solutionIndex); CHKERRQ(error);
	error = cg_field_write(file, base, zone, solutionIndex, CGNS_ENUMV(RealDouble), scalarFieldName.c_str(), scalarFieldRawData, &field); CHKERRQ(error);
	error = cg_close(file); CHKERRQ(error);
	return;
}

Eigen::VectorXd GridSave::readPermanentSolutionFromCGNSFile(const std::string cgnsFileName)
{
	Eigen::VectorXd permanentField;
	int error;
	int fileIndex=1, baseIndex=1, zoneIndex=1, solutionIndex=1;
	permanentField.resize(6);
	cgsize_t rangeMin=1, rangeMax=6;
	error = cg_open(cgnsFileName.c_str(), CG_MODE_READ, &fileIndex);
	error = cg_field_read(fileIndex, baseIndex, zoneIndex, solutionIndex, "Temperature", CGNS_ENUMV(RealDouble), &rangeMin, &rangeMax, &permanentField[0]);
	return permanentField;
}
