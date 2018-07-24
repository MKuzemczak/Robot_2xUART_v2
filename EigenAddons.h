#pragma once

#include <string>

#include "EigenLib/Eigen/Dense"
#include <EigenLib/Eigen/LU>
#include "uartCom.h"

#define SMALLEST 0.0000000001
#define DEG_TO_RAD 0.01745329251

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
		Eigen::Matrix <Scalar, ColsAtCompileTime, RowsAtCompileTime> transpose = m.transpose();
		Eigen::Matrix <Scalar, RowsAtCompileTime, RowsAtCompileTime> mult = m * transpose;
		Eigen::Matrix <Scalar, RowsAtCompileTime, RowsAtCompileTime> inv = mult.inverse();
		return transpose * inv;	
	}