#pragma once
// ============================================================
//  camcal.hpp : model kamera (piksel -> posisi di lantai) + kalibrasi otomatis
//  Lantai dianggap datar. Kamera di ketinggian h, menunduk sebesar pitch.
// ============================================================
#include <algorithm>
#include <cmath>

namespace nav {

// Piksel (u,v) -> posisi di frame robot (X depan, Y kiri). false bila di atas horizon.
inline bool pixelToFloor(double u, double v, double W, double H, double f,
                         double h, double pitch, double xoff, double& X, double& Y) {
  const double xn = (u - W * 0.5) / f;
  const double yn = (v - H * 0.5) / f;
  const double sp = std::sin(pitch), cp = std::cos(pitch);
  const double den = sp + yn * cp;
  if (den < 0.02) return false;
  const double t = h / den;
  X = t * (cp - yn * sp) + xoff;
  Y = -t * xn;
  return true;
}

struct CamFit { double h, pitch, rms; bool ok; };

// Cari (h, pitch) yang paling cocok dengan pasangan (baris-dasar-obstacle v, jarak sebenarnya d)
inline CamFit fitCamera(const double* v, const double* d, int n, double H, double f,
                        double xoff, double h0, double p0) {
  CamFit best; best.h = h0; best.pitch = p0; best.rms = 1e9; best.ok = false;
  if (n < 2) return best;
  double bestCost = 1e18;
  for (double pitch = -0.4; pitch <= 1.3; pitch += 0.004) {
    for (double h = 0.10; h <= 1.00; h += 0.004) {
      double cost = 0.0;
      for (int i = 0; i < n; i++) {
        const double yn = (v[i] - H * 0.5) / f;
        const double a = pitch + std::atan(yn);
        const double Xpred = (a > 0.02 ? h / std::tan(a) : 1e3) + xoff;
        const double e = (Xpred - d[i]) / std::max(0.2, d[i]);   // galat relatif
        cost += e * e;
      }
      if (cost < bestCost) { bestCost = cost; best.h = h; best.pitch = pitch; }
    }
  }
  best.rms = std::sqrt(bestCost / n);
  best.ok = true;
  return best;
}

}  // namespace nav
