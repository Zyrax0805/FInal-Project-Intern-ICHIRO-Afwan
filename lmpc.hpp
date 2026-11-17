#pragma once
// ============================================================
//  lmpc.hpp : Linear MPC (tanpa dependensi eksternal)
//  Model  : unicycle, dilinearkan di sekitar referensi (LTV)
//           e_{k+1} = A_k e_k + B_k du_k ,  e = [dx, dy, dtheta], du = u - u_ref
//  Biaya  : sum e'Qe + du'R du + (u_k - u_{k-1})'S(u_k - u_{k-1})
//  Batas  : 0 <= v <= V_MAX , |w| <= W_MAX
//  Solver : Fast projected gradient (FISTA) pada QP condensed
// ============================================================
#include <cmath>
#include <vector>
#include "config.hpp"
#include "davg.hpp"

namespace nav {

struct DMat {
  int r, c;
  std::vector<double> d;
  DMat() : r(0), c(0) {}
  DMat(int r_, int c_) : r(r_), c(c_), d((size_t)r_ * c_, 0.0) {}
  double& operator()(int i, int j) { return d[(size_t)i * c + j]; }
  double operator()(int i, int j) const { return d[(size_t)i * c + j]; }
};
inline DMat matMul(const DMat& A, const DMat& B) {
  DMat C(A.r, B.c);
  for (int i = 0; i < A.r; i++)
    for (int k = 0; k < A.c; k++) {
      const double a = A(i, k);
      if (a == 0.0) continue;
      for (int j = 0; j < B.c; j++) C(i, j) += a * B(k, j);
    }
  return C;
}
inline DMat matT(const DMat& A) {
  DMat T(A.c, A.r);
  for (int i = 0; i < A.r; i++)
    for (int j = 0; j < A.c; j++) T(j, i) = A(i, j);
  return T;
}

// pose = [x, y, theta] robot. Hasil: kecepatan linear v dan sudut w.
inline bool lmpcSolve(const double pose[3], const std::vector<Ref>& ref,
                      double uvPrev, double uwPrev, double& vOut, double& wOut) {
  const int n = MPC_N;
  if ((int)ref.size() < n) return false;
  const double dt = MPC_DT;

  // ---- prediksi: e_{1..n} = Phi*e0 + S*z ----
  DMat Phi(3 * n, 3), S(3 * n, 2 * n);
  for (int k = 0; k < n; k++) {
    const double c = std::cos(ref[k].th), s = std::sin(ref[k].th), vr = ref[k].v;
    double A[3][3] = {{1, 0, -vr * s * dt}, {0, 1, vr * c * dt}, {0, 0, 1}};
    double B[3][2] = {{c * dt, 0}, {s * dt, 0}, {0, dt}};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double acc = 0.0;
        for (int m = 0; m < 3; m++) {
          const double prev = (k == 0) ? (m == j ? 1.0 : 0.0) : Phi(3 * (k - 1) + m, j);
          acc += A[i][m] * prev;
        }
        Phi(3 * k + i, j) = acc;
      }
    if (k > 0)
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 2 * n; j++) {
          double acc = 0.0;
          for (int m = 0; m < 3; m++) acc += A[i][m] * S(3 * (k - 1) + m, j);
          S(3 * k + i, j) = acc;
        }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 2; j++) S(3 * k + i, 2 * k + j) = B[i][j];
  }

  double e0[3] = {pose[0] - ref[0].x, pose[1] - ref[0].y, wrapPi(pose[2] - ref[0].th)};
  std::vector<double> base(3 * n, 0.0), qd(3 * n, 0.0);
  for (int i = 0; i < 3 * n; i++)
    for (int j = 0; j < 3; j++) base[i] += Phi(i, j) * e0[j];
  for (int k = 0; k < n; k++) {
    const double sc = (k == n - 1) ? QF_SCALE : 1.0;
    qd[3 * k] = Q_X * sc; qd[3 * k + 1] = Q_Y * sc; qd[3 * k + 2] = Q_TH * sc;
  }

  // ---- bentuk QP: min 0.5 z'Hz + f'z ----
  const int m2 = 2 * n;
  DMat WS(3 * n, m2);                                   // diag(qd) * S
  for (int i = 0; i < 3 * n; i++)
    for (int j = 0; j < m2; j++) WS(i, j) = qd[i] * S(i, j);
  DMat H = matMul(matT(S), WS);
  for (int k = 0; k < n; k++) { H(2 * k, 2 * k) += R_V; H(2 * k + 1, 2 * k + 1) += R_W; }

  DMat D(m2, m2);                                       // selisih input berurutan
  std::vector<double> sd(m2), urs(m2, 0.0);
  for (int k = 0; k < n; k++) {
    D(2 * k, 2 * k) = 1.0; D(2 * k + 1, 2 * k + 1) = 1.0;
    if (k > 0) { D(2 * k, 2 * (k - 1)) = -1.0; D(2 * k + 1, 2 * (k - 1) + 1) = -1.0; }
    sd[2 * k] = S_V; sd[2 * k + 1] = S_W;
    urs[2 * k] = ref[k].v; urs[2 * k + 1] = 0.0;
  }
  DMat SD(m2, m2);
  for (int i = 0; i < m2; i++)
    for (int j = 0; j < m2; j++) SD(i, j) = sd[i] * D(i, j);
  DMat DtSD = matMul(matT(D), SD);
  for (int i = 0; i < m2; i++)
    for (int j = 0; j < m2; j++) H(i, j) += DtSD(i, j);

  std::vector<double> cvec(m2, 0.0);
  for (int i = 0; i < m2; i++)
    for (int j = 0; j < m2; j++) cvec[i] += D(i, j) * urs[j];
  cvec[0] -= uvPrev;
  cvec[1] -= uwPrev;

  std::vector<double> f(m2, 0.0), qb(3 * n), sc2(m2);
  for (int i = 0; i < 3 * n; i++) qb[i] = qd[i] * base[i];
  for (int i = 0; i < m2; i++) sc2[i] = sd[i] * cvec[i];
  for (int j = 0; j < m2; j++) {
    for (int i = 0; i < 3 * n; i++) f[j] += S(i, j) * qb[i];
    for (int i = 0; i < m2; i++) f[j] += D(i, j) * sc2[i];
  }

  // ---- batas input ----
  std::vector<double> lb(m2), ub(m2);
  for (int k = 0; k < n; k++) {
    lb[2 * k] = 0.0 - ref[k].v;      ub[2 * k] = V_MAX - ref[k].v;
    lb[2 * k + 1] = -W_MAX;          ub[2 * k + 1] = W_MAX;
  }

  // ---- konstanta Lipschitz (power iteration) ----
  std::vector<double> x(m2, 1.0 / std::sqrt((double)m2)), y(m2);
  double L = 1.0;
  for (int it = 0; it < 40; it++) {
    double nrm = 0.0;
    for (int i = 0; i < m2; i++) {
      double a = 0.0;
      for (int j = 0; j < m2; j++) a += H(i, j) * x[j];
      y[i] = a; nrm += a * a;
    }
    nrm = std::sqrt(nrm);
    if (nrm < 1e-12) break;
    L = nrm;                       // x bernorma 1, maka ||Hx|| -> lambda_max
    for (int i = 0; i < m2; i++) x[i] = y[i] / nrm;
  }
  L = L * 1.1 + 1e-9;

  // ---- FISTA dengan proyeksi ke kotak [lb, ub] ----
  std::vector<double> z(m2, 0.0), yv(m2, 0.0), zn(m2), g(m2);
  double tk = 1.0;
  for (int it = 0; it < QP_ITERS; it++) {
    for (int i = 0; i < m2; i++) {
      double a = f[i];
      for (int j = 0; j < m2; j++) a += H(i, j) * yv[j];
      g[i] = a;
    }
    for (int i = 0; i < m2; i++) zn[i] = clampd(yv[i] - g[i] / L, lb[i], ub[i]);
    const double tn = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * tk * tk));
    const double beta = (tk - 1.0) / tn;
    for (int i = 0; i < m2; i++) { yv[i] = zn[i] + beta * (zn[i] - z[i]); z[i] = zn[i]; }
    tk = tn;
  }

  // Humanoid: bila heading menyimpang jauh dari jalur, berhenti maju dan putar dulu
  const double hs = clampd(1.0 - (std::fabs(e0[2]) - 0.25) / 0.6, 0.0, 1.0);
  vOut = clampd((ref[0].v + z[0]) * hs, 0.0, V_MAX);
  wOut = clampd(z[1], -W_MAX, W_MAX);
  return true;
}

}  // namespace nav
