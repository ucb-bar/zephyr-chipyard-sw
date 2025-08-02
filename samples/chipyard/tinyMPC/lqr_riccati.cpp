#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <Eigen/Dense>
#include <fstream>
#include "gemmini.h"

using namespace Eigen;

static uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

void tiled_matmul_auto_eigen (
    const Matrix<float, Dynamic, Dynamic, RowMajor>&A,
    const Matrix<float, Dynamic, Dynamic, RowMajor>&B,
    Matrix<float, Dynamic, Dynamic, RowMajor>&C,
    bool transpose_A, bool transpose_B) 
{
        int i = transpose_A ? A.cols() : A.rows();
        int j = transpose_B ? B.rows() : B.cols();
        int k = transpose_B ? B.cols() : B.rows();
        tiled_matmul_auto(i, j, k,
                A.data(), B.data(), NULL, C.data(),
                transpose_A ? i : k, transpose_B ? k : j, j, j,
                MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
                NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
                transpose_A, transpose_B,
                false, false,
                0,
                WS
                );
}

void tiled_matmul_auto_eigen_bias (
    const Matrix<float, Dynamic, Dynamic, RowMajor>&A,
    const Matrix<float, Dynamic, Dynamic, RowMajor>&B,
    const Matrix<float, Dynamic, Dynamic, RowMajor>&D,
    Matrix<float, Dynamic, Dynamic, RowMajor>&C,
    bool transpose_A,
    bool transpose_B,
    bool sub ) 
{
        int i = transpose_A ? A.cols() : A.rows();
        int j = transpose_B ? B.rows() : B.cols();
        int k = transpose_B ? B.cols() : B.rows();
        tiled_matmul_auto(i, j, k,
                A.data(), B.data(), D.data() , C.data(),
                transpose_A ? i : k, transpose_B ? k : j, j, j,
                MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, sub ? -MVIN_SCALE_IDENTITY : MVIN_SCALE_IDENTITY,
                NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
                transpose_A, transpose_B,
                false, false,
                0,
                WS
                );
}


// Function to solve the finite-horizon discrete time LQR problem using Riccati recursion
Matrix<float, Dynamic, Dynamic, RowMajor> lqrSolveFiniteHorizon(const Matrix<float, Dynamic, Dynamic, RowMajor>& A, const Matrix<float, Dynamic, Dynamic, RowMajor>& B, const Matrix<float, Dynamic, Dynamic, RowMajor>& Q, const Matrix<float, Dynamic, Dynamic, RowMajor>& R, int horizon) {
    int state_dim = A.rows();
    int input_dim = B.cols();

    // Initialize the gain matrices
    std::vector<Matrix<float, Dynamic, Dynamic, RowMajor>> K(horizon, Matrix<float, Dynamic, Dynamic, RowMajor>::Zero(input_dim, state_dim));

    // Initialize the value function matrix
    Matrix<float, Dynamic, Dynamic, RowMajor> P = Q;

    // Perform the Riccati recursion
    for (int t = horizon - 1; t >= 0; --t) {
        Matrix<float, Dynamic, Dynamic, RowMajor> BRB = B.transpose() * P * B + R;
        Matrix<float, Dynamic, Dynamic, RowMajor> BRB_inv = BRB.inverse();
        K[t] = BRB_inv * B.transpose() * P * A;
        P = A.transpose() * P * A - A.transpose() * P * B * K[t] + Q;
    }

    return K[0];
}

// Function to solve the finite-horizon discrete time LQR problem using Riccati recursion
Matrix<float, Dynamic, Dynamic, RowMajor> lqrSolveFiniteHorizonLU(const Matrix<float, Dynamic, Dynamic, RowMajor>& A, const Matrix<float, Dynamic, Dynamic, RowMajor>& B, const Matrix<float, Dynamic, Dynamic, RowMajor>& Q, const Matrix<float, Dynamic, Dynamic, RowMajor>& R, int horizon) {
    int state_dim = A.rows();
    int input_dim = B.cols();

    // Initialize the gain matrices
    std::vector<Matrix<float, Dynamic, Dynamic, RowMajor>> K(horizon, Matrix<float, Dynamic, Dynamic, RowMajor>::Zero(input_dim, state_dim));

    // Initialize the value function matrix
    Matrix<float, Dynamic, Dynamic, RowMajor> P = Q;

    // Perform the Riccati recursion
    for (int t = horizon - 1; t >= 0; --t) {
        Matrix<float, Dynamic, Dynamic, RowMajor> BRB = B.transpose() * P * B + R;
        K[t] = BRB.partialPivLu().solve(B.transpose() * P * A);
        P = A.transpose() * P * A - A.transpose() * P * B * K[t] + Q;
    }

    return K[0];
}

Matrix<float, Dynamic, Dynamic, RowMajor> lqrSolveFiniteHorizonSingleOp(const Matrix<float, Dynamic, Dynamic, RowMajor>& A, const Matrix<float, Dynamic, Dynamic, RowMajor>& B, const Matrix<float, Dynamic, Dynamic, RowMajor>& Q, const Matrix<float, Dynamic, Dynamic, RowMajor>& R, int horizon) {
    const int state_dim = A.rows();
    const int input_dim = B.cols();

    // Initialize the gain matrices
    std::vector<Matrix<float, Dynamic, Dynamic, RowMajor>> K(horizon, Matrix<float, Dynamic, Dynamic, RowMajor>::Zero(input_dim, state_dim));

    // Initialize the value function matrix
    Matrix<float, Dynamic, Dynamic, RowMajor> P(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> PA(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> PB(state_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPB_R(input_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPA(input_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> APA(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPB_R_inv(input_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> APBK_Q(state_dim, state_dim);

    P = Q;
    // Perform the Riccati recursion
    for (int t = horizon - 1; t >= 0; --t) {
        PA = P * A;
        PB = P * B;
        BPB_R = B.transpose() * PB + R;
        BPA = B.transpose() * PA;
        APA = A.transpose() * PA;
        BPB_R_inv = BPB_R.inverse(); // CPU
        K[t] = BPB_R_inv * BPA;
        APBK_Q = BPA.transpose() * K[t] - Q;
        P = APA - APBK_Q;
    }
    return K[0];
}

Matrix<float, Dynamic, Dynamic, RowMajor> lqrSolveFiniteHorizonGemminiC(const Matrix<float, Dynamic, Dynamic, RowMajor>& A, const Matrix<float, Dynamic, Dynamic, RowMajor>& B, const Matrix<float, Dynamic, Dynamic, RowMajor>& Q, const Matrix<float, Dynamic, Dynamic, RowMajor>& R, int horizon) {
    const int state_dim = A.rows();
    const int input_dim = B.cols();

    // Initialize the gain matrices
    std::vector<Matrix<float, Dynamic, Dynamic, RowMajor>> K(horizon, Matrix<float, Dynamic, Dynamic, RowMajor>::Zero(input_dim, state_dim));

    Matrix<float, Dynamic, Dynamic, RowMajor> BT(input_dim, state_dim);
    BT = B.transpose().eval();
    Matrix<float, Dynamic, Dynamic, RowMajor> AT(input_dim, state_dim);
    AT = A.transpose().eval();
    Matrix<float, Dynamic, Dynamic, RowMajor> BPAT(state_dim, input_dim);

    // Initialize the value function matrix
    Matrix<float, Dynamic, Dynamic, RowMajor> P(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> PA(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> PB(state_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPB_R(input_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPA(input_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> APA(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPB_R_inv(input_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> APBK_Q(state_dim, state_dim);

    P = Q;
    // Perform the Riccati recursion
    for (int t = horizon - 1; t >= 0; --t) {
        tiled_matmul_auto_eigen(P, B, PB, false, false);
        tiled_matmul_auto_eigen_bias(B, PB, R, BPB_R, true, false, false);
        tiled_matmul_auto_eigen(P, A, PA, false, false);
        tiled_matmul_auto_eigen(B, PA, BPA, true, false);
        BPB_R_inv = BPB_R.inverse(); // CPU
        tiled_matmul_auto_eigen(BPB_R_inv, BPA, K[t], false, false);
        tiled_matmul_auto_eigen_bias(BPA, K[t], Q, APBK_Q, true, false, true);
        tiled_matmul_auto_eigen_bias(A, PA, APBK_Q, P, true, false, true);
    }
    return K[0];
}

Matrix<float, Dynamic, Dynamic, RowMajor> lqrSolveFiniteHorizonGemminiCLU(const Matrix<float, Dynamic, Dynamic, RowMajor>& A, const Matrix<float, Dynamic, Dynamic, RowMajor>& B, const Matrix<float, Dynamic, Dynamic, RowMajor>& Q, const Matrix<float, Dynamic, Dynamic, RowMajor>& R, int horizon) {
    const int state_dim = A.rows();
    const int input_dim = B.cols();

    // Initialize the gain matrices
    std::vector<Matrix<float, Dynamic, Dynamic, RowMajor>> K(horizon, Matrix<float, Dynamic, Dynamic, RowMajor>::Zero(input_dim, state_dim));

    Matrix<float, Dynamic, Dynamic, RowMajor> BT(input_dim, state_dim);
    BT = B.transpose().eval();
    Matrix<float, Dynamic, Dynamic, RowMajor> AT(input_dim, state_dim);
    AT = A.transpose().eval();
    Matrix<float, Dynamic, Dynamic, RowMajor> BPAT(state_dim, input_dim);

    // Initialize the value function matrix
    Matrix<float, Dynamic, Dynamic, RowMajor> P(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> PA(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> PB(state_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPB_R(input_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPA(input_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> APA(state_dim, state_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> BPB_R_inv(input_dim, input_dim);
    Matrix<float, Dynamic, Dynamic, RowMajor> APBK_Q(state_dim, state_dim);

    P = Q;
    // Perform the Riccati recursion
    for (int t = horizon - 1; t >= 0; --t) {
        tiled_matmul_auto_eigen(P, B, PB, false, false);
        tiled_matmul_auto_eigen_bias(B, PB, R, BPB_R, true, false, false);
        tiled_matmul_auto_eigen(P, A, PA, false, false);
        tiled_matmul_auto_eigen(B, PA, BPA, true, false);
        K[t] = BPB_R.partialPivLu().solve(BPA);
        tiled_matmul_auto_eigen_bias(BPA, K[t], Q, APBK_Q, true, false, true);
        tiled_matmul_auto_eigen_bias(A, PA, APBK_Q, P, true, false, true);
    }
    return K[0];
}


// int main(int argc, char* argv[]) {
//     if (argc != 4) {
//         std::cerr << "Usage: " << argv[0] << " <state_space_size> <action_space_size> <horizon_length>" << std::endl;
//         return 1;
//     }

//     int state_dim = std::stoi(argv[1]);
//     int input_dim = std::stoi(argv[2]);
//     int horizon   = std::stoi(argv[3]);

//     uint64_t t0;
//     uint64_t t1;
//     uint64_t time1;
//     uint64_t time2;
//     uint64_t time3;
//     uint64_t time4;

//     // Initialize the random number generator
//     std::srand(static_cast<unsigned>(std::time(0)));

//     // Define the system matrices A and B with random values
//     Matrix<float, Dynamic, Dynamic, RowMajor> A(state_dim, state_dim);
//     A.setRandom();
//     Matrix<float, Dynamic, Dynamic, RowMajor> B(state_dim, input_dim);
//     B.setRandom();

//     // Define the cost matrices Q and R with random values
//     Matrix<float, Dynamic, Dynamic, RowMajor> Q(state_dim, state_dim);
//     Q.setRandom();
//     Q = Q.transpose() * Q; // Make Q symmetric and positive semi-definite
//     Matrix<float, Dynamic, Dynamic, RowMajor> R(input_dim, input_dim);
//     R.setRandom();
//     R = R.transpose() * R; // Make R symmetric and positive definite


//     t0 = read_cycles();
//     Matrix<float, Dynamic, Dynamic, RowMajor> K = lqrSolveFiniteHorizon(A, B, Q, R, horizon);
//     t1 = read_cycles();
//     time1 = t1 - t0;

//     t0 = read_cycles();
//     Matrix<float, Dynamic, Dynamic, RowMajor> K2 = lqrSolveFiniteHorizonLU(A, B, Q, R, horizon);
//     t1 = read_cycles();
//     time2 = t1 - t0;

//     t0 = read_cycles();
//     Matrix<float, Dynamic, Dynamic, RowMajor> K3 = lqrSolveFiniteHorizonGemminiCLU(A, B, Q, R, horizon);
//     t1 = read_cycles();
//     time3 = t1 - t0;

//     std::cout << "Optimal gain matrix K:  (" << time1 << " )" << std::endl;
//     // std::cout << K << std::endl;

//     std::cout << "Optimal gain matrix K2: (" << time2 << " )" << std::endl;
//     if(!K.isApprox(K2)) {
//         std::cout << "ERROR: Differing Results" << std::endl;
//         std::cout << "K" << std::endl;
//         std::cout << K << std::endl;
//         std::cout << "K2" << std::endl;
//         std::cout << K2 << std::endl;
//     }

//     std::cout << "Optimal gain matrix K3: (" << time3 << " )" << std::endl;
//     if(!K3.isApprox(K2)) {
//         std::cout << "ERROR: Differing Results" << std::endl;
//         std::cout << "K2" << std::endl;
//         std::cout << K2 << std::endl;
//         std::cout << "K3" << std::endl;
//         std::cout << K3 << std::endl;
//     }

//     return 0;
// }

Matrix<float, Dynamic, Dynamic, RowMajor> randomStableMatrix(int size) {
    Matrix<float, Dynamic, Dynamic, RowMajor> D = Matrix<float, Dynamic, Dynamic, RowMajor>::Identity(size, size);
    for (int i = 0; i < size; ++i) {
        D(i, i) = 0.8 * static_cast<float>(rand()) / RAND_MAX;
    }

    Matrix<float, Dynamic, Dynamic, RowMajor> P = Matrix<float, Dynamic, Dynamic, RowMajor>::Random(size, size);
    Matrix<float, Dynamic, Dynamic, RowMajor> A = P.inverse() * D * P;

    return A;
}

Matrix<float, Dynamic, Dynamic, RowMajor> randomPositiveSemidefiniteMatrix(int size) {
    Matrix<float, Dynamic, Dynamic, RowMajor> M = Matrix<float, Dynamic, Dynamic, RowMajor>::Random(size, size);
    Matrix<float, Dynamic, Dynamic, RowMajor> Q = M.transpose() * M;
    return Q;
}

Matrix<float, Dynamic, Dynamic, RowMajor> randomPositiveDefiniteMatrix(int size) {
    Matrix<float, Dynamic, Dynamic, RowMajor> M = Matrix<float, Dynamic, Dynamic, RowMajor>::Random(size, size);
    Matrix<float, Dynamic, Dynamic, RowMajor> R = M.transpose() * M;
    for (int i = 0; i < size; ++i) {
        R(i, i) += 1.0;
    }
    return R;
}

int main() {
    // csv_file << "state_space_size,action_space_size,horizon_length,time2,time3\n";
    std::cout << "state_space_size,action_space_size,horizon_length,time2,time3,time4,error_norm,error_norm_LU,error_norm_gemmini\n";

    // std::vector<int> dimensions = {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128};
    // std::vector<int> dimensions = {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64};
    // std::vector<int> dimensions = {1, 2, 3, 7, 8, 9};
    std::vector<int> dimensions = {4, 8};

    // int horizon = 8;
    int horizon = 2;

    // for (int input_dim = 1; input_dim <= 48; input_dim++) {
    //     for (int state_dim = 1; state_dim <= 48; state_dim++) {
    for (int input_dim : dimensions) {
        for (int state_dim : dimensions) {
            uint64_t t0, t1, time2, time3, time4;

            // Initialize the random number generator
            std::srand(static_cast<unsigned>(std::time(0)));

            // Define the system matrices A and B with random values
            // Matrix<float, Dynamic, Dynamic, RowMajor> A(state_dim, state_dim);
            // A.setRandom();
            // Matrix<float, Dynamic, Dynamic, RowMajor> B(state_dim, input_dim);
            // B.setRandom();

            // // Define the cost matrices Q and R with random values
            // Matrix<float, Dynamic, Dynamic, RowMajor> Q(state_dim, state_dim);
            // Q.setRandom();
            // Q = Q.transpose() * Q; // Make Q symmetric and positive semi-definite
            // Matrix<float, Dynamic, Dynamic, RowMajor> R(input_dim, input_dim);
            // R.setRandom();
            // R = R.transpose() * R; // Make R symmetric and positive definite

            // Define the system matrices A and B with random values
            Matrix<float, Dynamic, Dynamic, RowMajor> A = randomStableMatrix(state_dim);
            Matrix<float, Dynamic, Dynamic, RowMajor> B(state_dim, input_dim);
            B.setRandom();

            // Define the cost matrices Q and R with random values
            Matrix<float, Dynamic, Dynamic, RowMajor> Q = randomPositiveDefiniteMatrix(state_dim);
            Matrix<float, Dynamic, Dynamic, RowMajor> R = randomPositiveSemidefiniteMatrix(input_dim);

            t0 = read_cycles();
            Matrix<float, Dynamic, Dynamic, RowMajor> K2 = lqrSolveFiniteHorizonLU(A, B, Q, R, horizon);
            t1 = read_cycles();
            time2 = t1 - t0;

            t0 = read_cycles();
            Matrix<float, Dynamic, Dynamic, RowMajor> K3 = lqrSolveFiniteHorizonGemminiC(A, B, Q, R, horizon);
            t1 = read_cycles();
            time3 = t1 - t0;

            t0 = read_cycles();
            Matrix<float, Dynamic, Dynamic, RowMajor> K4 = lqrSolveFiniteHorizonGemminiCLU(A, B, Q, R, horizon);
            t1 = read_cycles();
            time4 = t1 - t0;


            float error_norm = (K2 - K3).norm();
            float error_norm_LU = (K2 - K4).norm();
            float error_norm_gemmini = (K3 - K4).norm();
            std::cout << state_dim << "," << input_dim << "," << horizon << "," << time2 << "," << time3 << "," << time4 << "," << error_norm << "," << error_norm_LU << ',' << error_norm_gemmini << "\n";
        }
    }

    return 0;
}

