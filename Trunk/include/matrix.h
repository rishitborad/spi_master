/**********************************************************************************
* File name:  matrix.h
*
* File description: 
*   - Header file for matrix math routines.  Definitions of the matrix structure
*	  and the matric block structure are here.
*               
* $Rev: 17024 $
* $Date: 2010-11-23 19:43:03 -0800 (Tue, 23 Nov 2010) $
* $Author: by-lfera $
*
**********************************************************************************/
#ifndef MATRIX_H
#define MATRIX_H

typedef struct _mat{
	int m; /* number of rows */
	int n; /* number of columns */
	double *Ptr;  /* pointer to matrix values, going accross each row left to right */
}matrixStruct;

typedef struct _matblk{
	matrixStruct M;         /* full matrix */
	int mtype;      		/* row block size     */
	int ntype;      		/* column block size  */
	int *typePtr;   		/* pointer to block type matrix        */
	int *mSizePtr;  		/* pointer to row block size matrix    */
	int *nSizePtr;  		/* pointer to column block size matrix */
}matrixBlockStruct;

extern void  mat_move_block(matrixStruct ,matrixBlockStruct ,int ,int);
extern void  mat_move_matblock(matrixBlockStruct A,matrixBlockStruct *B);
extern void  mat_transpose_block(matrixBlockStruct ,matrixBlockStruct *);
extern void  mat_multiply_block(matrixBlockStruct ,matrixBlockStruct ,matrixBlockStruct *,int,int *);

extern void  mat_zero(matrixStruct);
extern void  mat_make_identity(matrixStruct);
extern void  mat_subtract(matrixStruct ,matrixStruct, matrixStruct *,int *);
extern void  mat_add(matrixStruct ,matrixStruct ,matrixStruct *,int *);
extern void  mat_multiply(matrixStruct ,matrixStruct ,matrixStruct *,int *);
extern void  mat_multiply_invert( matrixStruct , matrixStruct , int *);
extern void  mat_move(matrixStruct,matrixStruct *);
extern void  mat_multiply_constant(double ,matrixStruct ,matrixStruct *);
extern void  mat_transpose(matrixStruct ,matrixStruct *);
extern void max_value_in_vector(matrixStruct a,double *val,int *index);

#endif
