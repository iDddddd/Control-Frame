#pragma once


class Matrix
{
public:
    float** data;  // ��ά������Ϊ��Ա����
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

