#include <Utils/ExportConvergenceToCSV.hpp>

void createConvergenceFile(const std::string& fileName)
{
	std::ofstream csvFile;
	csvFile.open(fileName, std::ios::out);
	csvFile.close();
	return;
}

void appendConvergenceToCSV(const std::string& fileName, const std::vector<double>& error, const std::vector<double>& characteristicLength)
{
	std::ofstream csvFile;
	csvFile.open(fileName, std::ios::app);
	// characteristic length
	for(unsigned count=0 ; count<(characteristicLength.size()-1) ; ++count)
	{
		csvFile << doubleToString(characteristicLength[count]) << ",";
	}
		csvFile << doubleToString(characteristicLength.back());
		csvFile << std::endl;
	// error
	for(unsigned count=0 ; count<(error.size()-1) ; ++count)
	{
		csvFile << doubleToString(error[count]) << ",";
	}
		csvFile << doubleToString(error.back());
	csvFile << std::endl;
	csvFile.close();
	return;
}