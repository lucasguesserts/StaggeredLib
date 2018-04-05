#include <Utils/EigenTest.hpp>

bool operator==(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
	return lhs.coeff(0)==Approx(rhs.coeff(0)) &&
	       lhs.coeff(1)==Approx(rhs.coeff(1)) &&
	       lhs.coeff(2)==Approx(rhs.coeff(2));
}

bool operator==(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs)
{
	bool vality = true;
	if(lhs.size()==rhs.size())
		for(unsigned entry=0 ; entry<lhs.size() ; ++entry)
			vality = vality && lhs.coeff(entry)==Approx(rhs.coeff(entry));
	else
		vality = false;
	return vality;
}

bool operator==(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs)
{
	bool vality = true;
	if((lhs.rows()==rhs.rows()) && (lhs.cols()==rhs.cols()))
		for(unsigned row=0 ; row<lhs.rows() ; ++row)
			for(unsigned column=0 ; column<lhs.cols() ; ++column)
				vality = vality && lhs.coeff(row,column)==Approx(rhs.coeff(row,column));
	else
		vality = false;
	return vality;
}

std::string rowToString(const std::string& initialChars, const Eigen::MatrixXd& matrix, const unsigned row, const std::string& finalChars)
{
	// numberString: "+1.1234567890e+123, " -> 18+2 char
	// row format: " 2.1234567890e+385, 8.9437813657e-028,\n" -> initialChars = " ", finalChars = ",\n"
	unsigned column;
	std::string str;
	str += initialChars;
	for(column=0 ; column<(matrix.cols()-1) ; ++column)
		str += (boost::format("%10.10le, ") % matrix.coeff(row,column)).str();
	column = matrix.cols()-1;
		str += (boost::format("%10.10le%s") % matrix.coeff(row,column) % finalChars).str();
	return str;
}