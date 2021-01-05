#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <mpi.h>
#include "mat.h"
#include "vec.h"
#include "utils.h"

/* y <- Ax
 * - A: matrix
 * - x: input vector
 * - y: output vector
 */
int MatMult(Mat A, Vec x, Vec y)
{
  int ierr;
  
  if (A->N != x->N || A->N != y->N || x->n != A->n/A->np || x->n != y->n) {
    fprintf(stderr, "Mismatching sizes in MatMult %d %d %d\n", A->N, x->N, y->N);
    return MPI_Abort(A->comm, MPI_ERR_ARG);
  }
  fprintf(stderr, "[MatMult]: TODO, please implement me.\n");
  /* Do local part of multiplication. This is only correct in serial.
   * This code is included to show you how to call MatMultLocal,
   * you'll need to change the arguments in parallel.
   */
  ierr = MatMultLocal(x->n, A->data, x->data, y->data);CHKERR(ierr);
  return 0;
}

/* C <- AB + C using the SUMMA algorithm.
 *
 * - A: input matrix
 * - B: input matrix
 * - C: output matrix
 */
int MatMatMultSumma(Mat A, Mat B, Mat C)
{
  int ierr;
  fprintf(stderr, "[MatMatMultSumma]: TODO, please implement me.\n");
  /* Do local part of multiplication. Only correct in serial. */
  ierr = MatMatMultLocal(A->n, A->data, B->data, C->data);CHKERR(ierr);
  return 0;
}

/* C <- AB + C using Cannon's algorithm.
 *
 * - A: input matrix
 * - B: input matrix
 * - C: output matrix
 */
void naive_multiply_add(int size, double* A, double* B, double* C)
{
    int i, j, k;
    for ( i = 0; i < size; ++i) {
        for ( j = 0; j < size; ++j) {
            double s = 0;
            for ( k = 0; k < size; ++k) {
                s += A[i * size + k] * B[k * size + j];
            }
            C[i * size + j] += s;
        }
    }
}
void Cannon(int N,double* A, double* B, double* C) 
{
    // save the id of the current process and the total number of processes
    int num_proc, my_id;
    MPI_Comm_size(MPI_COMM_WORLD, &num_proc);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_id);

    // amount of blocks in one row / column of the matrix
    const int grid_size = (int)sqrt(num_proc);
    // amount of columns / rows in one matrix block (pad with zeros)
    const int extra_space = N % grid_size ? grid_size - (N % grid_size) : 0;
    const int block_size = (N + extra_space) / grid_size;
    // number of elements in one matrix block
    const int local_matrix_size = block_size * block_size;


    // create 2D sqrt(p) x sqrt(p) grid communicator
    const int periods[2] = { 1, 1 }; // periodic in both dimensions
    const int dims[2] = { grid_size, grid_size }; // size of each dimension
    MPI_Comm grid_comm;
    MPI_Cart_create(MPI_COMM_WORLD, 2, dims, periods, 0, &grid_comm);

    // local matrices
    double* local_A = (double*)malloc(sizeof(double)*local_matrix_size);
    double* local_B = (double*)malloc(sizeof(double) * local_matrix_size);
    double* local_C = (double*)malloc(sizeof(double) * local_matrix_size);
    // fill matrices with zeros before adding values to it
    int i, j, k;
    for (i = 0; i < local_matrix_size; i++) {
        local_A[i] = 0;
        local_B[i] = 0;
        local_C[i] = 0;
    }
 
    // get coordinates of process inside the 2D grid
    int coords[2] = { 0, 0 };
    MPI_Cart_coords(grid_comm, my_id, grid_size, coords);
    // all indices smaller than this number need to send less data
    const int adjust_indices = (grid_size - (N % grid_size)) % grid_size;
    // datatypes and counts + displacements for send and receive
    MPI_Datatype TMP,
        BLOCK[num_proc], // send data from A or B to local matrix
        LOCAL_BLOCK[num_proc], // Receive local_A or local_B from A or B
        GATHER_LOCAL[num_proc]; // Gather local_C in C
    int* displs =(int*)malloc(sizeof(int)*num_proc);
    int row_offset = 0, col_offset = 0;
    int row, col;
    for (row = 0; row < grid_size; ++row) {
        col_offset = 0;
        const int off_row = (row < adjust_indices);
        for (col = 0; col < grid_size; ++col) {
            // ignore padding for first columns when sending the data
            const int off_col = (col < adjust_indices);
            const int p_id = row * grid_size + col;

            // create datatype to send submatrices (BLOCK)
            MPI_Type_vector(block_size - off_row, block_size - off_col, N, MPI_DOUBLE, &TMP);
            MPI_Type_create_resized(TMP, 0, sizeof(double), &BLOCK[p_id]);
            MPI_Type_commit(&BLOCK[p_id]);

            // create datatype to receive submatrices (LOCAL_BLOCK)
            MPI_Type_vector(block_size - off_row, block_size - off_col, block_size, MPI_DOUBLE, &TMP);
            MPI_Type_create_resized(TMP, 0, sizeof(double), &LOCAL_BLOCK[p_id]);
            MPI_Type_commit(&LOCAL_BLOCK[p_id]);

            // create datatype to receive submatrices in C (GATHER_LOCAL)
            MPI_Type_vector(block_size - off_row, block_size - off_col, N, MPI_DOUBLE, &TMP);
            MPI_Type_create_resized(TMP, 0, sizeof(double), &GATHER_LOCAL[p_id]);
            MPI_Type_commit(&GATHER_LOCAL[p_id]);

            // displacement for send operation
            displs[p_id] = row_offset + col_offset;
            // increase column offset
            col_offset += block_size - off_col;
        }
        // increase row offset
        row_offset += N * (block_size - off_row);
    }

    // scatter data so that each process (i,j) has the entries for the
    // matrix block at index (i, j)
    MPI_Request req;
    if (!my_id) {
        for (i = 0; i < num_proc; ++i) {
            MPI_Isend(A + displs[i], 1, BLOCK[i], i, 0, grid_comm, &req);
            MPI_Isend(B + displs[i], 1, BLOCK[i], i, 1, grid_comm, &req);
        }
    }
    MPI_Recv(local_A, 1, LOCAL_BLOCK[my_id], 0, 0, grid_comm, MPI_STATUS_IGNORE);
    MPI_Recv(local_B, 1, LOCAL_BLOCK[my_id], 0, 1, grid_comm, MPI_STATUS_IGNORE);

    // wait until all data is send
    MPI_Barrier(grid_comm);
    /*
    After inital scatter:
    Example blocks for matrix A or B (the numbers denote the process ids):
    0,0  |  0, 1  |  0, 2
    1,0  |  1, 1  |  1, 2
    2,0  |  2, 1  |  2, 2
    */

    // ids of neighbours
    int right = 0, left = 0, down = 0, up = 0;
    // shift A based on the row and B based on the current column
    MPI_Cart_shift(grid_comm, 1, coords[0], &left, &right);
    MPI_Cart_shift(grid_comm, 0, coords[1], &up, &down);
    MPI_Sendrecv_replace(local_A, local_matrix_size, MPI_DOUBLE, left,0, right, 0, grid_comm, MPI_STATUS_IGNORE);
    MPI_Sendrecv_replace(local_B, local_matrix_size, MPI_DOUBLE, up,0, down, 0, grid_comm, MPI_STATUS_IGNORE);

    /*
    After inital shift:
    Example blocks for matrix A:       Example blocks for matrix B:
    0,0  |  0, 1  |  0, 2              0,0  |  1, 1  |  2, 2
    1,1  |  1, 2  |  1, 0              1,0  |  2, 1  |  0, 2
    2,2  |  2, 0  |  2, 1              2,0  |  0, 1  |  1, 2
    */

    // multiply and add values to C = C + A*B
    naive_multiply_add(block_size, local_A, local_B, local_C);

    // shift values of A/B to left/up in a loop
    for (i = 1; i < grid_size; ++i) {

        MPI_Cart_shift(grid_comm, 1, 1, &left, &right);
        MPI_Cart_shift(grid_comm, 0, 1, &up, &down);
        MPI_Sendrecv_replace(local_A, local_matrix_size, MPI_DOUBLE, left,0, right, 0, grid_comm, MPI_STATUS_IGNORE);
        MPI_Sendrecv_replace(local_B, local_matrix_size, MPI_DOUBLE, up, 0, down, 0, grid_comm, MPI_STATUS_IGNORE);
        naive_multiply_add(block_size, local_A, local_B, local_C);
    }

    // gather values in C for final result
    MPI_Isend(local_C, 1, LOCAL_BLOCK[my_id], 0, 0, grid_comm, &req);
    if (!my_id) {
        for ( i = 0; i < num_proc; ++i) {
            MPI_Recv(C + displs[i], 1, GATHER_LOCAL[i], i, 0, grid_comm,MPI_STATUS_IGNORE);
        }
    }

    MPI_Barrier(grid_comm);

    // cleanup
    free(displs);
    free(local_A);
    free(local_B);
    free(local_C);
}

int MatMatMultCannon(Mat A, Mat B, Mat C)
{
    int ierr;
    fprintf(stderr, "[MatMatMultCannon]: TODO, please implement me.\n");

    // save the id of the current process and the total number of processes
    int num_proc, my_id;
    MPI_Comm_size(MPI_COMM_WORLD, &num_proc);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_id);
  

    // create 2D sqrt(p) x sqrt(p) grid communicator
    int i;
    int N = A->N;
    double* global_A=(double*)malloc(sizeof(double) * N*N);
    double* global_B = (double*)malloc(sizeof(double) * N*N);
    double* global_C = (double*)malloc(sizeof(double) * N*N);
    if (my_id == 0) {
        for (i = 0; i < N * N; i++) {
            global_A[i] = A->data[i];
            global_B[i] = B->data[i];
            global_C[i] = C->data[i];
        }

    }
    MPI_Barrier(MPI_COMM_WORLD);
    Cannon(N,global_A, global_B, global_C);
    
    if (my_id == 0) {
        for (i = 0; i < N * N; i++) {
            C->data[i] += global_C[i];
        }
    }
 
    /* Do local part of multiplication. Only correct in serial. */
    ierr = MatMatMultLocal(A->n, A->data, B->data, C->data); CHKERR(ierr);
    return 0;
}
