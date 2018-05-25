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

bool operator==(const Eigen::SparseMatrix<double>& lhs, const Eigen::SparseMatrix<double>& rhs)
{
	bool vality = true;
	if ((lhs.rows()==rhs.rows()) && (lhs.cols()==rhs.cols()) && (lhs.nonZeros()==rhs.nonZeros()))
	{
		for (unsigned column=0; column<lhs.outerSize(); ++column)
		{
			Eigen::SparseMatrix<double>::InnerIterator lhsIterator(lhs,column);
			Eigen::SparseMatrix<double>::InnerIterator rhsIterator(rhs,column);
			for( ; lhsIterator && rhsIterator ; ++lhsIterator, ++rhsIterator)
			{
				vality = vality && lhsIterator.value()==Approx(rhsIterator.value());
				vality = vality && lhsIterator.row()==Approx(rhsIterator.row());
				vality = vality && lhsIterator.col()==Approx(rhsIterator.col());
			}
			if(!vality) break;
		}
	}
	else
		vality = false;
	return vality;
}

std::string eigenVector3dToString(const Eigen::Vector3d& vector)
{
	// vector format: "[+2.1234567890e+385, -8.9437813657e-028, +5.3716842318e+008]"
	std::string str;
	str += "[";
	str += doubleToString(vector.coeff(0)) + ",";
	str += doubleToString(vector.coeff(1)) + ",";
	str += doubleToString(vector.coeff(2)) + "]";
	return str;
}

std::string eigenVectorToString(const Eigen::VectorXd& vector)
{
	// vector format: "[+2.1234567890e+385, -8.9437813657e-028]"
	unsigned entry;
	std::string str;
	str += "[";
	for(entry=0 ; entry<(vector.size()-1) ; ++entry)
		str += doubleToString(vector.coeff(entry)) + ",";
	entry = vector.size()-1;
		str += doubleToString(vector.coeff(entry)) + "]";
	return str;
}

std::string eigenMatrixRowToString(const std::string& initialChars, const Eigen::MatrixXd& matrix, const unsigned row, const std::string& finalChars)
{
	// numberString: "+1.1234567890e+123, " -> 18+2 char
	// row format: " -2.1234567890e+385, +8.9437813657e-028,\n" -> initialChars = " ", finalChars = ",\n"
	unsigned column;
	std::string str;
	str += initialChars;
	for(column=0 ; column<(matrix.cols()-1) ; ++column)
		str += doubleToString(matrix.coeff(row,column)) + ", ";
	column = matrix.cols()-1;
		str += doubleToString(matrix.coeff(row,column)) + finalChars;
	return str;
}

std::string eigenMatrixToString(const Eigen::MatrixXd& matrix)
{
	// Format:
	// [2.4000000000e+00, 1.0000000000e+00,
	//  8.6000000000e+00, 5.7000000000e+00,
	//  4.1000000000e+00, 9.0000000000e-01]
	std::string str;
	unsigned row;
	row = 0;
		str += eigenMatrixRowToString("[", matrix, row, ",\n");
	for(row=1 ; row<(matrix.rows()-1) ; ++row)
		str += eigenMatrixRowToString(" ", matrix, row, ",\n");
	row = matrix.rows()-1;
		str += eigenMatrixRowToString(" ", matrix, row, "]");
	return str;
}

std::string eigenSparseMatrixToString(const Eigen::SparseMatrix<double>& matrix)
{
	// format: { (row_0, column_0, value_0), (row_1, column_1, value_1), ..., (row_n, column_n, value_n) }
	std::string str;
	str += "{ ";
	for (unsigned column=0; column<matrix.outerSize(); ++column)
	{
		for(Eigen::SparseMatrix<double>::InnerIterator it(matrix,column) ; it ; ++it)
		{
			str += "(";
			str += std::to_string(it.row()) + ", ";
			str += std::to_string(it.col()) + ", ";
			str += doubleToString(it.value());
			str += ")";
		}
	}
	str += " }";
	return str;
}