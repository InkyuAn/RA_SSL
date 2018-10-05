#include <string.h>
#include <float.h>
#include <math.h>
#include <iostream>

#define MAX_ITERATION_COUNT 30   // Maximum number of iterations

void testPrint();

void Householders_Reduction_to_Bidiagonal_Form(double* A, int nrows, int ncols, double* U, double* V, double* diagonal, double* superdiagonal);

int  Givens_Reduction_to_Diagonal_Form(int nrows, int ncols, double* U, double* V, double* diagonal, double* superdiagonal);

void Sort_by_Decreasing_Singular_Values(int nrows, int ncols, double* singular_value, double* U, double* V);

int Singular_Value_Decomposition(double* A, int nrows, int ncols, double* U, double* singular_values, double* V, double* dummy_array);