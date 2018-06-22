#ifndef EXPORT_CONVERGENCE_TO_CSV
#define EXPORT_CONVERGENCE_TO_CSV

#include <vector>
#include <string>
#include <fstream>
#include <Utils/String.hpp>

void createConvergenceFile(const std::string& fileName);
void appendConvergenceToCSV(const std::string& fileName, const std::vector<double>& error, const std::vector<double>& characteristicLength);

#endif