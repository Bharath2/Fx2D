#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <iostream>
#include <random>

int main(){
    constexpr int N    = 100;      // how many 2-vectors
    constexpr int REPS = 1000000;     // how many times to repeat

    // ─── set up RNG ────────────────────────────────────────
    std::mt19937                               rng{123};
    std::uniform_real_distribution<float> dist{-1.0f,1.0f};

    // ─── 1) fill a std::vector of Vector2f ────────────────
    std::vector<Eigen::Vector2f> vecs(N);
    for(int i=0;i<N;++i)
        vecs[i] = Eigen::Vector2f{ dist(rng), dist(rng) };

    // ─── 2) pack them into an N×2 matrix (row-major!) ────
    Eigen::Matrix<float, N, 2, Eigen::RowMajor> mat;
    for(int i=0;i<N;++i)
        mat.row(i) = vecs[i];

    // ─── the 2×2 transform ────────────────────────────────
    Eigen::Matrix2f M;
    M << dist(rng), dist(rng),
         dist(rng), dist(rng);

    // ─── warm up ───────────────────────────────────────────
    Eigen::Vector2f tmpv = M * vecs[0];
    Eigen::Matrix<float,N,2> tmpm = mat * M.transpose();

    // ─── benchmark A: looped transforms ───────────────────
    auto t0 = std::chrono::high_resolution_clock::now();
    float sinkA = 0.0f;
    std::vector<Eigen::Vector2f> outv(N);
    for(int rep=0;rep<REPS;++rep){
        for(int i=0;i<N;++i){
            outv[i] = M * vecs[i];
        }
        // touch first & last so compiler can’t elide
        sinkA += outv[0].x() + outv[N-1].y();
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double durA = std::chrono::duration<double>(t1-t0).count();

    // ─── benchmark B: big mat×mat multiply ────────────────
    auto t2 = std::chrono::high_resolution_clock::now();
    float sinkB = 0.0f;
    Eigen::Matrix<float, N, 2> outm;
    for(int rep=0;rep<REPS;++rep){
        // each row of (mat * Mᵀ) is (M * vec)ᵀ
        outm.noalias() = mat * M.transpose();
        sinkB += outm(0,0) + outm(N-1,1);
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    double durB = std::chrono::duration<double>(t3-t2).count();

    // ─── report ────────────────────────────────────────────
    std::cout
      << "sinkA: " << sinkA << "\n"
      << "sinkB: " << sinkB << "\n"
      << "N = " << N << ", REPS = " << REPS << "\n"
      << "A (loop):        " << durA << " s\n"
      << "B (matxmat):     " << durB << " s\n"
      << "speedup A/B:     " << (durA/durB) << "x\n";
}
