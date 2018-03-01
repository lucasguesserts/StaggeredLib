#ifndef CGNS_FILE_HPP
#define CGNS_FILE_HPP

#include <string>
#include <vector>

class CGNSFile
{
	public:
		CGNSFile(const std::string cgnsFileName);
		std::vector<double> readCoordinate(const std::string& coordinateName);

		int fileIndex, zoneIndex, baseIndex;
		int cellDimension, physicalDimension; // read in base
		unsigned numberOfVertices, numberOfElements; // read in zone

	private:
		// TODO: add exceptions.
		void openFile(const std::string cgnsFileName);
		void openBase(void);
		void openZone(void);
		void verifyNumberOfGrids(void);
		void verifyNumberOfCoordinates(void);
};

#endif
