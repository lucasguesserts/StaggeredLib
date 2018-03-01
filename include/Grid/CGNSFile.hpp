#ifndef CGNS_FILE_HPP
#define CGNS_FILE_HPP

#include <string>

class CGNSFile
{
	public:
		CGNSFile(const std::string cgnsFileName);

		int fileIndex, zoneIndex, baseIndex;
		int cellDimension, physicalDimension; // read in base
		unsigned numberOfVertices, numberOfElements; // read in zone

	private:
		void openFile(const std::string cgnsFileName);
		void openBase(void);
		void openZone(void);
};

#endif
