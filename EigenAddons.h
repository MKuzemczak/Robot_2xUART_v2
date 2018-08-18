#pragma once

#include <string>

#include "EigenLib/Eigen/Dense"
#include <EigenLib/Eigen/LU>
#include "uartCom.h"
#include "Lista.h"

#define SMALLEST 0.0000000001
#define DEG_TO_RAD 0.01745329251
#define PI 3.14159265359

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
	UartPort & operator <<(UartPort & port, Eigen::Matrix< Scalar, RowsAtCompileTime, ColsAtCompileTime> &m)
	{
		std::string s;
		
		for (int i = 0, rows = m.rows(), cols = m.cols(); i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				s.append(std::to_string(m(i,j)));
				s.append("  ");
			}
			s.append("\n");
		}
		
		port << s;
		
		return port;
	}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
	void printNumbers(UartPort & port, Eigen::Matrix< Scalar, RowsAtCompileTime, ColsAtCompileTime> &m)
	{
		std::string s;
		
		for (int i = 0, rows = m.rows(), cols = m.cols(); i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				s.append(std::to_string(m(i, j)));
				s.append("\n");
			}
		}
		
		port << s;
	}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
	Eigen::Matrix <Scalar, ColsAtCompileTime, RowsAtCompileTime> pseudoInverse(Eigen::Matrix< Scalar, RowsAtCompileTime, ColsAtCompileTime> &m)
	{
		Lista<int> zeroCols, zeroRows;
		
		bool check = true;
		
		
		/////////////////////////////////////////// Finding zero rows & columns
		// pcPort << "Finding zero rows & columns\n"; //debug
		for (int i = 0; i < m.rows(); i++)
		{
			for(int j = 0; j < m.cols(); j++)
				if(fabs(m(i,j)) > SMALLEST)
					check = false;
			
			if (check)
				zeroRows.push_back(i);
			
			check = true;
		}
		
		if (zeroRows.size() == m.rows())
			return Eigen::Matrix <Scalar, ColsAtCompileTime, RowsAtCompileTime>();
		
		for (int i = 0; i < m.cols(); i++)
		{
			for (int j = 0; j < m.rows(); j++)
				if (fabs(m(j, i)) > SMALLEST)
					check = false;
			
			if (check)
				zeroCols.push_back(i);
			
			check = true;
		}
		// pcPort << "~Finding zero rows & columns\n"; //dubug
		/////////////////////////////////////////// ~Finding zero rows & columns
		
		// Dimensions of matrix w/o zero rows & columns
		int newRows = m.rows() - (int)zeroRows.size(),
			newCols = m.cols() - (int)zeroCols.size();
		
		//pcPort << "Dimensions of matrix w/o zero rows & columns\n"; //debug
		//pcPort << "newRows: " << newRows << ", newCols: " << newCols << '\n'; //debug
		//pcPort << "RowsAtCompileTime: " << RowsAtCompileTime << ", zerRows.size(): " << (int)zeroRows.size() << '\n'; //debug
		
		// Matrix without zero rows & columns
		Eigen::MatrixXd m0(newRows, newCols);
		
		//pcPort << "Matrix without zero rows & columns\n"; // debug
		
		
		// vars helping in filling m0 matrix
		int rows = 0, cols = 0;
		
		//pcPort << "vars helping in filling m0 matrix\n"; //debug
		
		
		//pcPort << "Filling m0 matrix w/ remaining rows&cols\n"; //debug
		
		////////////////////////////////////////// Filling m0 matrix w/ remaining rows&cols
		for (int i = 0; i < m.rows(); i++)
			for (int j = 0; j < m.cols(); j++)
			{
				for (int k = 0; k < zeroRows.size(); k++)
					if (zeroRows[k] == i)
						check = false;
				
				for (int k = 0; k < zeroCols.size(); k++)
					if (zeroCols[k] == j)
						check = false;
				
				if (check)
				{
					m0(rows, cols++) = m(i, j);
					
					if (cols == m0.cols())
					{
						cols = 0;
						rows++;
					}
				}
				
				check = true;
			}
		////////////////////////////////////////// ~Filling m0 matrix w/ remaining rows&cols
		
		// pcPort << "~Filling m0 matrix w/ remaining rows&cols\n"; //debug
		
		// pcPort << "Calculations according to formula\n"; //debug
		
		///////////////////////////////////////////// Calculations according to formula
		Eigen::MatrixXd transpose(newCols, newRows);
		transpose = m0.transpose();
		
		//pcPort << "Transpose:\n" << transpose << '\n'; //debug
		
		Eigen::MatrixXd mult(newRows, newRows);
		mult = m0 * transpose;
		
		// pcPort << "mult:\n" << mult << '\n'; //debug
		
		Eigen::MatrixXd inv(newRows, newRows);
		inv = mult.inverse();
		
		// pcPort << "inv:\n" << inv << '\n'; //debug
		
		Eigen::MatrixXd result(newCols, newRows);
		result = transpose * inv;
		
		// pcPort << "result:\n" << result << '\n'; //debug
		
		
		///////////////////////////////////////////// ~Calculations according to formula
		
		//pcPort << "~Calculations according to formula\n"; //debug
		
		
		//pcPort << "\"Putting back\" transposed zero rows&cols in the result of calculations\n"; //debug
		
		////////////////////////////////////////////////////////////////// "Putting back" transposed zero rows&cols in the result of calculations
		Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(m.cols(), m.rows());
		
		rows = 0;
		cols = 0;
		
		for (int i = 0; i < ret.rows(); i++)
		{
			for (int j = 0; j < ret.cols(); j++)
			{
				for (int k = 0; k < zeroCols.size(); k++)
					if (zeroCols[k] == i)
						check = false;
				
				for (int k = 0; k < zeroRows.size(); k++)
					if (zeroRows[k] == j)
						check = false;
				
				if (check)
				{
					ret(i, j) = result(rows, cols++);
					if (cols == newRows)
					{
						cols = 0;
						rows++;
					}
				}
				
				check = true;
				
			}
			// pcPort << "\"Putting back\" loop\n"; //debug
		}
		////////////////////////////////////////////////////////////////// ~"Putting back" transposed zero rows&cols in the result of calculations
		
		// pcPort << "~\"Putting back\" transposed zero rows&cols in the result of calculations\n"; //debug
		
		return ret;	
	}