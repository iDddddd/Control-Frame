#include "Matrix.h"

Matrix::Matrix(int numRows, int numCols) {
	rows = numRows;
	cols = numCols;
	data = new float* [rows];
	for (int i = 0; i < rows; ++i) {
		data[i] = new float[cols];
		for (int j = 0; j < cols; ++j) {
			data[i][j] = 0.0;
		}
	}
}

Matrix::Matrix(float* inputData, int numRows, int numCols) {
	rows = numRows;
	cols = numCols;
	data = new float* [rows];
	for (int i = 0; i < rows; ++i) {
		data[i] = new float[cols];
		for (int j = 0; j < cols; ++j) {
			data[i][j] = inputData[i*cols+j];
		}
	}
}

Matrix::~Matrix() {
	for (int i = 0; i < rows; ++i) {
		delete[] data[i];
	}
	delete[] data;
}

void Matrix::operator=(const Matrix& other) const {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			data[i][j] = other.data[i][j];
		}
	}
}

Matrix Matrix::operator+(const Matrix& other) const {
	if (rows != other.rows || cols != other.cols) {
		//exit(1);
	}

	Matrix result(rows, cols);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result.data[i][j] = data[i][j] + other.data[i][j];
		}
	}
	return result;
}

Matrix Matrix::operator-(const Matrix& other) const {
	if (rows != other.rows || cols != other.cols) {
		//exit(1);
	}

	Matrix result(rows, cols);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result.data[i][j] = data[i][j] - other.data[i][j];
		}
	}
	return result;
}

// 数乘
Matrix Matrix::operator*(float scalar) const {
	Matrix result(rows, cols);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result.data[i][j] = data[i][j] * scalar;
		}
	}
	return result;
}

Matrix Matrix::operator*(const Matrix& other) const {
	if (cols != other.rows) {
		//exit(1);
	}

	Matrix result(rows, other.cols);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < other.cols; ++j) {
			for (int k = 0; k < cols; ++k) {
				result.data[i][j] += data[i][k] * other.data[k][j];
			}
		}
	}
	return result;
}

Matrix Matrix::transpose() const {
	Matrix result(cols, rows);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result.data[j][i] = data[i][j];
		}
	}
	return result;
}



   // 求逆
   Matrix Matrix::inverse() const {
	   if (rows != cols) {
		   //exit(1);
	   }
	   int n = rows;
	   float** augmentedMatrix = new float* [n];
	   for (int i = 0; i < n; ++i) {
		   augmentedMatrix[i] = new float[2 * n];
		   for (int j = 0; j < n; ++j) {
			   augmentedMatrix[i][j] = data[i][j];
			   
		   }
		   for (int j = 0; j < n; j++) {
			   if(j == i)
				   augmentedMatrix[i][j + n] = 1.0;
			   else
				   augmentedMatrix[i][j + n] = 0.0;
		   }
		   
	   }
	   // 高斯-约当消元法求逆矩阵
	   for (int i = 0; i < n; ++i) {
		   // 将第i列的主元素移到对角线上
		   if (augmentedMatrix[i][i] == 0.0) {
			   int j = i + 1;
			   while (j < n && augmentedMatrix[j][i] == 0.0) {
				   ++j;
			   }
			   if (j == n) {
				   //exit(1);
			   }
			   //std::swap(augmentedMatrix[i], augmentedMatrix[j]);
			   int k;
			   for (k = 0; k < 10; k++) {
				   float tmp = augmentedMatrix[i][k];
				   augmentedMatrix[i][k] = augmentedMatrix[j][k];
				   augmentedMatrix[j][k] = tmp;
			   }
		   }

		   float pivot = augmentedMatrix[i][i];
		   for (int j = 0; j < 2 * n; ++j) {
			   augmentedMatrix[i][j] /= pivot;
		   }

		   // 将第i列的其他元素变为零
		   for (int j = 0; j < n; ++j) {
			   if (j != i) {
				   float ratio = augmentedMatrix[j][i];
				   for (int k = 0; k < 2 * n; ++k) {
					   augmentedMatrix[j][k] -= ratio * augmentedMatrix[i][k];
				   }
			   }
		   }
	   }

	   // 提取逆矩阵
	   Matrix result(n, n);
	   for (int i = 0; i < n; ++i) {
		   for (int j = 0; j < n; ++j) {
			   result.data[i][j] = augmentedMatrix[i][j + n];
		   }
	   }

	   for (int i = 0; i < n; ++i) {
		   delete[] augmentedMatrix[i];
	   }
	   delete[] augmentedMatrix;

	   return result;
   }



