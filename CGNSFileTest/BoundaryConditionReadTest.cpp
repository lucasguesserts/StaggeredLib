#include <Utils/Test.hpp>
#include <string>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>

TestCase("cgns read boundary", "[CGNSFile]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "CGNSFile_boundary_read_test.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	constexpr int numberOfBoundaries = 4;
	require(numberOfBoundaries==cgnsFile.readNumberOfBoundaries());
	section("botton boundary")
	{
		int boundaryIndex = 1;
		const std::vector<unsigned> boundaryElementList = {6, 7};
		check(boundaryElementList==cgnsFile.readBoundaryElementList(boundaryIndex));
	}
}