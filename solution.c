#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <mpi.h>
#include "mat.h"
#include "vec.h"
#include "utils.h"

void MatrixMultiply(double* A, double* B, double* C, int m, int n, int p)
{
    int i, j, k;
    for (i = 0; i < m; i++) {
        for (j = 0; j < p; j++) {
            double result = 0;
            for (k = 0; k < n; k++) {
                result = A[i * n + k] * B[k * p + j] + result;

            }
            C[i * p + j] = result;
        }
    }
}
void MatrixAdd(double* A, double* B, int m, int n) //the result remain in A
{
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            A[i * n + j] = A[i * n + j] + B[i * n + j];
        }
    }
}

/* y <- Ax
 * - A: matrix
 * - x: input vector
 * - y: output vector
 */
int MatMult(Mat A, Vec x, Vec y)
{
   // n*n×n*1=n*1
    int num_proc, myid;
    MPI_Status status;
    MPI_Comm_size(MPI_COMM_WORLD, &num_proc);
    MPI_Comm_rank(MPI_COMM_WORLD, &myid);
    int i, j, k;
    int p = A->np;
    int myRow = myid / p;
    int myCol = myid % p;
    if (myid == 1) {
        printf("A\n");
        k = 0;
        for (i = 0; i < A->n; i++) {
            for (j = 0; j < A->n; j++) {
                printf("%lf\t",A->data[k++]);
            }
        }
        printf("x\n");
        k = 0;
        for (i = 0; i < x->n; i++) {
            printf("%lf\t", x->data[k++]);
        }
        printf("\n");
    }
    for (i = 0; i < p; i++) {
        MPI_Send(A->data, A->n * A->n, MPI_DOUBLE, myRow * p + i, 1, MPI_COMM_WORLD);
        MPI_Send(x->data, x->n , MPI_DOUBLE, myRow * p + i, 2, MPI_COMM_WORLD);

        MPI_Send(A->data, A->n * A->n, MPI_DOUBLE, i * p + myCol, 1, MPI_COMM_WORLD);
        MPI_Send(x->data, x->n, MPI_DOUBLE, i * p + myCol, 2, MPI_COMM_WORLD);

    }
    double* recvA = (double*)malloc(sizeof(double) * A->n * A->n);
    double* recvx = (double*)malloc(sizeof(double) * x->n);
    double* resultC = (double*)malloc(sizeof(double) * y->n);

    for (i = 0; i < y->n ; i++) {
        resultC[i] = 0;
    }
    //计算矩阵的值
    for (i = 0; i < p; i++) {
        MPI_Recv(recvA, A->n * A->n, MPI_DOUBLE, myRow * p + i, 1, MPI_COMM_WORLD, &status);
        MPI_Recv(recvx, x->n, MPI_DOUBLE, i * p + myCol, 2, MPI_COMM_WORLD, &status);
        MatrixMultiply(recvA, recvx, resultC, A->n, A->n, 1);
        MatrixAdd(y->data, resultC, A->n, 1);
    }
  
  return 0;
}

void naive_multiply_add(int size, double* A, double* B, double* C)
{
    int i, j, k;
    for (i = 0; i < size; ++i) {
        for (j = 0; j < size; ++j) {
            double s = 0;
            for (k = 0; k < size; ++k) {
                s += A[i * size + k] * B[k * size + j];
            }
            C[i * size + j] += s;
        }
    }
}
/* C <- AB + C using the SUMMA algorithm.
 *
 * - A: input matrix
 * - B: input matrix
 * - C: output matrix
 */
int MatMatMultSumma(Mat A, Mat B, Mat C)
{
  int num_proc, myid;
  MPI_Status status;
  MPI_Comm_size(MPI_COMM_WORLD, &num_proc);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
  int i, j, k;
  int p = A->np;
  int myRow = myid / p;
  int myCol = myid % p;
  for (i = 0; i < p; i++) {
      MPI_Send(A->data, A->n * A->n, MPI_DOUBLE, myRow * p + i, 1, MPI_COMM_WORLD);
      MPI_Send(B->data, B->n * B->n, MPI_DOUBLE, myRow * p + i, 2, MPI_COMM_WORLD);

      MPI_Send(A->data, A->n * A->n, MPI_DOUBLE, i * p + myCol, 1, MPI_COMM_WORLD);
      MPI_Send(B->data, B->n * B->n, MPI_DOUBLE, i * p + myCol, 2, MPI_COMM_WORLD);

  }
  double* recvA = (double*)malloc(sizeof(double) * A->n * A->n);
  double* recvB = (double*)malloc(sizeof(double) * B->n * B->n);
  double* resultC = (double*)malloc(sizeof(double) * C->n * C->n);

  for (i = 0; i < A->n * B->n; i++) {
      resultC[i] = 0;
  }
  //计算矩阵的值
  for (i = 0; i < p; i++) {
      MPI_Recv(recvA, A->n * A->n, MPI_DOUBLE, myRow *p + i, 1, MPI_COMM_WORLD,&status);
      MPI_Recv(recvB, B->n * B->n, MPI_DOUBLE, i * p + myCol, 2, MPI_COMM_WORLD,&status);
      naive_multiply_add(A->n, recvA, recvB, C->data);
  }
  return 0;
}

/* C <- AB + C using Cannon's algorithm.
 *
 * - A: input matrix
 * - B: input matrix
 * - C: output matrix
 */

/*
 *@row:矩阵所在的行
 *@col:矩阵所在的列
 *@sp:sp=root=sqrt(nprocs)
 *@return 根据行列号计算进程实际编号
*/
int get_index(int row, int col, int sp) {
    int tmp = ((row + sp) % sp) * sp + ((col + sp) % sp);
    return tmp;
}
/*用于矩阵下标定位对齐*/
void shuffle(double* A, double* buf_A, int buf_A_size, double* B, double* buf_B, int buf_B_size, int root, int myid) {
    int i, j;
    MPI_Status status;
    int cur_col = 0;
    int cur_row = 0;
    /*通过进程编号计算获得当前进程所在的行号和列号*/
    cur_row = myid / root;
    //cur_col = myid - cur_row * root;
    cur_col = myid%root;
    /*对于矩阵A，第i行的矩阵需要向左平移i次*/
    for (i = 0; i < cur_row; i++) {
        /*接收来自右边的数据，并将当前矩阵发送给左边的进程*/
        MPI_Sendrecv(A, buf_A_size, MPI_DOUBLE, get_index(cur_row, cur_col - 1, root), 102,buf_A, buf_A_size, MPI_DOUBLE, get_index(cur_row, cur_col + 1, root), 102, MPI_COMM_WORLD, &status);
        memcpy(A, buf_A, buf_A_size * sizeof(double));/*buf_A用于通信时缓存矩阵*/
        memset(buf_A, 0, buf_A_size * sizeof(double));
    }
    /*对于矩阵B，第j列的矩阵需要向上平移j次*/
    for (j = 0; j < cur_col; j++) {
        /*接收来自下边的数据，并将当前矩阵发送给上边的进程*/
        MPI_Sendrecv(B, buf_B_size, MPI_DOUBLE, get_index(cur_row - 1, cur_col, root), 103,buf_B, buf_B_size, MPI_DOUBLE, get_index(cur_row + 1, cur_col, root), 103, MPI_COMM_WORLD, &status);
        memcpy(B, buf_B, buf_B_size * sizeof(double));/*buf_B用于通信时缓存矩阵*/
        memset(buf_B, 0, buf_B_size * sizeof(double));
    } 
}
/*计算矩阵乘法，将结果存入C中*/
void matrix_multi(double* A, double* B, double* C, int n, int myid) {
    int i = 0, j = 0, k = 0;
    double s = 0;
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            s = 0;
            for (k = 0; k < n; k++) {
                //tmp_C[i * n + j] += A[i * n + k] * B[k * n + j];
                s += A[i * n + k] * B[k * n + j];
            }
            C[i * n + j] += s;
        }

    }
}
void cannon(double* A, double* buf_A, int buf_A_size, double* B, double* buf_B, int buf_B_size,
    double* C, int buf_C_size, int n, int root, int myid) {
    
    MPI_Status status;
    int i, j;
    //memset(C, 0, sizeof(double) * buf_C_size);
    int cur_col = 0;
    int cur_row = 0;
    /*通过进程编号计算获得当前进程所在的行号和列号*/
    cur_row = myid / root;
    cur_col = myid - cur_row * root;

    for (i = 0; i < root; i++) {/*一共需要循环root次，root=sqrt(nprocs)*/
        matrix_multi(A, B, C, n, myid);/*计算矩阵乘法*/
        /*接收来自右边(row,col+1)的数据，并将当前矩阵发送给左边(row,col-1)的进程*/
        MPI_Sendrecv(A, buf_A_size, MPI_DOUBLE, get_index(cur_row, cur_col - 1, root), 102,
            buf_A, buf_A_size, MPI_DOUBLE, get_index(cur_row, cur_col + 1, root), 102, MPI_COMM_WORLD, &status);
        /*接收来自下边(row+1,col)的数据，并将当前矩阵发送给上边(row-1,col)的进程*/
        MPI_Sendrecv(B, buf_B_size, MPI_DOUBLE, get_index(cur_row - 1, cur_col, root), 103,
            buf_B, buf_B_size, MPI_DOUBLE, get_index(cur_row + 1, cur_col, root), 103, MPI_COMM_WORLD, &status);
       
        memcpy(B, buf_B, buf_B_size * sizeof(double));/*将buf_B中的数据拷贝至B中*/
        memcpy(A, buf_A, buf_A_size * sizeof(double));/*将buf_A中的数据拷贝至A中*/

    }
}

int MatMatMultCannon(Mat A, Mat B, Mat C)
{
    int ierr,i,j;
    int num_proc, myid;
    MPI_Comm_size(MPI_COMM_WORLD, &num_proc);
    MPI_Comm_rank(MPI_COMM_WORLD, &myid);
    int n = A->n;
    int root = A->np;
    if (root * root != num_proc) {
        printf("process number must be a squre!\n");
        return 0;
    }
    
    double * buf_A, * buf_B;
    buf_A = (double*)malloc(sizeof(double) *n*n);
    buf_B = (double*)malloc(sizeof(double) *n*n);
    if (buf_A == NULL || buf_B == NULL) {
        printf("Memory allocation failed!\n");
        return 0;
    }
    int buf_A_size = A->n * A->n;
    int buf_B_size = B->n * B->n;
    int buf_C_size = C->n * C->n;

    /*compute C=C+A*B by Cannon algorithm*/
     /*矩阵块必须定位对齐，先做预处理*/
    MPI_Barrier(MPI_COMM_WORLD);
   
    shuffle(A->data, buf_A, buf_A_size, B->data, buf_B, buf_B_size, root, myid);
    MPI_Barrier(MPI_COMM_WORLD);/*等待所有进程完成cannon算法,*/
   
    cannon(A->data, buf_A, buf_A_size, B->data, buf_B, buf_B_size, C->data, buf_C_size, A->n, root, myid);
   
   
    free(buf_B);
    free(buf_A);
    MPI_Barrier(MPI_COMM_WORLD);/*等待所有进程完成cannon算法,*/

    return 0;
}
