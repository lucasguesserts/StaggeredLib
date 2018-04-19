#include <Utils/Test.hpp>
#include <string>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>

TestCase("cgns transform indices from CGNS to my structure", "[CGNSFile]")
{
	const std::vector<unsigned> indices = {2, 8, 21};
	const std::vector<cgns::cgsize_t>& indicesToTransform = {3, 9, 22};
	std::vector<unsigned> transformedIndices = CGNSFile::transformCGNSIndices(indicesToTransform);
	check(indices==transformedIndices);
}

TestCase("cgns read boundary elements list", "[CGNSFile]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "CGNSFile_boundary_read_test.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	constexpr cgns::cgsize_t numberOfElements = 2;
	constexpr unsigned boundaryIndex = 2;
	std::vector<unsigned> elementIndices = {8, 9};
	check(elementIndices==cgnsFile.readBoundaryElementList(boundaryIndex, numberOfElements));
}

TestCase("cgns read boundary", "[CGNSFile][BoundaryDefinition]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "CGNSFile_boundary_read_test.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	std::vector<BoundaryDefinition> boundaryVector = cgnsFile.readBoundaries();
	constexpr int numberOfBoundaries = 4;
	require(numberOfBoundaries==cgnsFile.readNumberOfBoundaries());
	section("bottom boundary")
	{
		constexpr int boundaryIndex = 1;
		std::string boundaryName = "bottom boundary";
		const std::vector<unsigned> boundaryElementList = {6, 7};
		BoundaryDefinition& boundary = boundaryVector[0];
		check(boundaryIndex==boundary.index);
		check(boundaryName==std::string(boundary.name));
		check(boundaryElementList==boundary.elementsIndexList);
	}
	section("top boundary")
	{
		constexpr int boundaryIndex = 2;
		std::string boundaryName = "top boundary";
		const std::vector<unsigned> boundaryElementList = {8, 9};
		BoundaryDefinition& boundary = boundaryVector[1];
		check(boundaryIndex==boundary.index);
		check(boundaryName==std::string(boundary.name));
		check(boundaryElementList==boundary.elementsIndexList);
	}
	section("west boundary")
	{
		constexpr int boundaryIndex = 3;
		std::string boundaryName = "west boundary";
		const std::vector<unsigned> boundaryElementList = {10, 11};
		BoundaryDefinition& boundary = boundaryVector[2];
		check(boundaryIndex==boundary.index);
		check(boundaryName==std::string(boundary.name));
		check(boundaryElementList==boundary.elementsIndexList);
	}
	section("east boundary")
	{
		constexpr int boundaryIndex = 4;
		std::string boundaryName = "east boundary";
		const std::vector<unsigned> boundaryElementList = {12, 13};
		BoundaryDefinition& boundary = boundaryVector[3];
		check(boundaryIndex==boundary.index);
		check(boundaryName==std::string(boundary.name));
		check(boundaryElementList==boundary.elementsIndexList);
	}
}