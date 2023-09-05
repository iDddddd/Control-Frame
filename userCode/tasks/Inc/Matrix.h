#pragma once


class Matrix
{
public:
    float** data;  // 二维数组作为成员变量
    int rows;
    int cols;


    Matrix(int numRows, int numCols);//
    Matrix(float* inputData, int numRows, int numCols);
    ~Matrix();
    void operator=(const Matrix& other) const;
    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(float scalar) const;
    Matrix operator*(const Matrix& other) const;
    Matrix transpose() const;
    Matrix inverse() const;
};

