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
	constexpr int numberOfBoundaries = 4;
	std::vector<BoundaryDefinition> boundaryVector = cgnsFile.readBoundaries();
	require(numberOfBoundaries==cgnsFile.readNumberOfBoundaries());
	section("botton boundary")
	{
		std::string boundaryName = "bottom boundary";
		const std::vector<unsigned> boundaryElementList = {6, 7};
		BoundaryDefinition boundary = getBoundaryDefinitionInVector(boundaryName, boundaryVector);
		check(boundaryElementList==boundary.elementsIndexList);
	}
	section("top boundary")
	{
		std::string boundaryName = "top boundary";
		const std::vector<unsigned> boundaryElementList = {8, 9};
		BoundaryDefinition boundary = getBoundaryDefinitionInVector(boundaryName, boundaryVector);
		check(boundaryElementList==boundary.elementsIndexList);
	}
	section("west boundary")
	{
		std::string boundaryName = "west boundary";
		const std::vector<unsigned> boundaryElementList = {10, 11};
		BoundaryDefinition boundary = getBoundaryDefinitionInVector(boundaryName, boundaryVector);
		check(boundaryElementList==boundary.elementsIndexList);
	}
	section("east boundary")
	{
		std::string boundaryName = "east boundary";
		const std::vector<unsigned> boundaryElementList = {12, 13};
		BoundaryDefinition boundary = getBoundaryDefinitionInVector(boundaryName, boundaryVector);
		check(boundaryElementList==boundary.elementsIndexList);
	}
	section("error")
	{
		std::string boundaryName = "inexistent boundary";
		const std::vector<unsigned> boundaryElementList = {3, 6};
		try
		{
			BoundaryDefinition boundary = getBoundaryDefinitionInVector(boundaryName, boundaryVector);
			check(false);
		}
		catch (const std::runtime_error& error)
		{
			check(true);
		}
	}
}