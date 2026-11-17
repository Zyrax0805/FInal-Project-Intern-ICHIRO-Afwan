#pragma once
// ============================================================
//  davg.hpp : Dynamic Augmented Visibility Graph (tanpa dependensi)
//   - Augmented : obstacle digembungkan (radius robot + margin)
//   - Visibility graph : node = robot, goal, sudut obstacle
//   - Dynamic   : graf dibangun ulang dari peta obstacle terbaru
// ============================================================
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "config.hpp"

namespace nav {

struct Pt     { double x, y; };
struct Obs    { double cx, cy, half; };          // obstacle (persegi, frame global)
struct Rect   { double xmin, xmax, ymin, ymax; };
struct Ref    { double x, y, th, v; };           // titik referensi LMPC
struct Detect { double X, Y, half, u, v; };      // deteksi di frame robot (X depan, Y kiri) + piksel dasar

inline double wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

inline Rect inflateObs(const Obs& o, double extra) {
  Rect r;
  r.xmin = o.cx - o.half - extra;  r.xmax = o.cx + o.half + extra;
  r.ymin = o.cy - o.half - extra;  r.ymax = o.cy + o.half + extra;
  return r;
}
inline bool insideRect(const Pt& p, const Rect& r) {
  return p.x > r.xmin && p.x < r.xmax && p.y > r.ymin && p.y < r.ymax;
}

// Apakah segmen a-b memotong BAGIAN DALAM persegi r (Liang-Barsky)
inline bool segHitsRect(const Pt& a, const Pt& b, const Rect& r) {
  double t0 = 0.0, t1 = 1.0;
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double p[4] = {-dx, dx, -dy, dy};
  const double q[4] = {a.x - r.xmin, r.xmax - a.x, a.y - r.ymin, r.ymax - a.y};
  for (int i = 0; i < 4; i++) {
    if (std::fabs(p[i]) < 1e-12) {
      if (q[i] <= 0.0) return false;
    } else {
      const double t = q[i] / p[i];
      if (p[i] < 0.0) { if (t > t1) return false; if (t > t0) t0 = t; }
      else            { if (t < t0) return false; if (t < t1) t1 = t; }
    }
  }
  return (t1 - t0) > 1e-9;
}

inline bool visibleSeg(const Pt& a, const Pt& b, const std::vector<Rect>& rects) {
  for (size_t i = 0; i < rects.size(); i++)
    if (segHitsRect(a, b, rects[i])) return false;
  return true;
}

// Bila titik berada di dalam zona terlarang, dorong keluar lewat sisi terdekat
inline Pt pushOut(Pt p, const std::vector<Rect>& rects) {
  for (size_t i = 0; i < rects.size(); i++) {
    const Rect& r = rects[i];
    if (!insideRect(p, r)) continue;
    const double dl = p.x - r.xmin, dr = r.xmax - p.x;
    const double db = p.y - r.ymin, du = r.ymax - p.y;
    const double m = std::min(std::min(dl, dr), std::min(db, du));
    if (m == dl)      p.x = r.xmin - 0.005;
    else if (m == dr) p.x = r.xmax + 0.005;
    else if (m == db) p.y = r.ymin - 0.005;
    else              p.y = r.ymax + 0.005;
  }
  return p;
}

// ---------- Perencana DAVG: kembalikan polyline start -> goal ----------
inline std::vector<Pt> davgPlan(Pt start, Pt goal, const std::vector<Obs>& map, bool& ok) {
  std::vector<Rect> test, node;
  for (size_t i = 0; i < map.size(); i++) {
    test.push_back(inflateObs(map[i], ROBOT_R + VIS_EXTRA));
    node.push_back(inflateObs(map[i], ROBOT_R + SAFETY_MARGIN));
  }
  start = pushOut(start, test);
  goal  = pushOut(goal, test);

  std::vector<Pt> nodes;
  nodes.push_back(start);
  nodes.push_back(goal);
  for (size_t i = 0; i < node.size(); i++) {
    const Pt cs[4] = {{node[i].xmin, node[i].ymin}, {node[i].xmin, node[i].ymax},
                      {node[i].xmax, node[i].ymin}, {node[i].xmax, node[i].ymax}};
    for (int c = 0; c < 4; c++) {
      if (std::fabs(cs[c].y) > ARENA_Y_LIM) continue;
      bool bad = false;
      for (size_t j = 0; j < test.size(); j++)
        if (j != i && insideRect(cs[c], test[j])) { bad = true; break; }
      if (!bad) nodes.push_back(cs[c]);
    }
  }

  const int n = (int)nodes.size();
  const double INF = std::numeric_limits<double>::max();
  std::vector<double> dist(n, INF);
  std::vector<int> prev(n, -1);
  std::vector<char> done(n, 0);
  dist[0] = 0.0;
  for (int it = 0; it < n; it++) {
    int u = -1;
    for (int i = 0; i < n; i++)
      if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
    if (u < 0 || dist[u] >= INF) break;
    done[u] = 1;
    if (u == 1) break;
    for (int v = 0; v < n; v++) {
      if (done[v]) continue;
      if (!visibleSeg(nodes[u], nodes[v], test)) continue;
      const double d = dist[u] + std::hypot(nodes[u].x - nodes[v].x, nodes[u].y - nodes[v].y);
      if (d < dist[v]) { dist[v] = d; prev[v] = u; }
    }
  }

  std::vector<Pt> path;
  if (dist[1] >= INF) {           // tidak ada jalur: kembali ke garis lurus
    ok = false;
    path.push_back(start);
    path.push_back(goal);
    return path;
  }
  ok = true;
  for (int c = 1; c != -1; c = prev[c]) path.push_back(nodes[c]);
  std::reverse(path.begin(), path.end());
  return path;
}

// ---------- Referensi LMPC: sampel N titik sepanjang polyline ----------
inline void locateOnPath(const std::vector<Pt>& path, const std::vector<double>& cum,
                         double s, double& x, double& y, double& th) {
  const size_t m = path.size();
  size_t i = 0;
  while (i + 2 < m && s > cum[i + 1]) i++;
  const Pt& a = path[i];
  const Pt& b = path[i + 1];
  const double len = cum[i + 1] - cum[i];
  const double t = (len > 1e-9) ? clampd((s - cum[i]) / len, 0.0, 1.0) : 0.0;
  x = a.x + t * (b.x - a.x);
  y = a.y + t * (b.y - a.y);
  th = std::atan2(b.y - a.y, b.x - a.x);
}

inline std::vector<Ref> sampleReference(const std::vector<Pt>& path, const Pt& robot,
                                        const Pt& goal) {
  std::vector<Ref> ref;
  const size_t m = path.size();
  std::vector<double> cum(m, 0.0);
  for (size_t i = 1; i < m; i++)
    cum[i] = cum[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  const double total = (m > 0) ? cum[m - 1] : 0.0;

  if (m < 2 || total < 1e-6) {
    Ref r;
    r.x = robot.x; r.y = robot.y;
    r.th = std::atan2(goal.y - robot.y, goal.x - robot.x);
    r.v = 0.0;
    for (int k = 0; k < MPC_N; k++) ref.push_back(r);
    return ref;
  }

  // proyeksi robot ke polyline
  double best = std::numeric_limits<double>::max(), s0 = 0.0;
  for (size_t i = 0; i + 1 < m; i++) {
    const double ax = path[i].x, ay = path[i].y;
    const double bx = path[i + 1].x - ax, by = path[i + 1].y - ay;
    const double len2 = bx * bx + by * by;
    if (len2 < 1e-12) continue;
    const double t = clampd(((robot.x - ax) * bx + (robot.y - ay) * by) / len2, 0.0, 1.0);
    const double d = std::hypot(robot.x - (ax + t * bx), robot.y - (ay + t * by));
    if (d < best) { best = d; s0 = cum[i] + t * std::sqrt(len2); }
  }

  double s = s0;
  for (int k = 0; k < MPC_N; k++) {
    Ref r;
    locateOnPath(path, cum, std::min(s, total), r.x, r.y, r.th);
    const double remaining = std::max(0.0, total - s);
    r.v = std::min(V_REF, std::max(0.25 * V_REF, 0.8 * remaining));
    ref.push_back(r);
    s += r.v * MPC_DT;
  }
  return ref;
}

// ---------- Peta obstacle persisten (fusi deteksi kamera) ----------
// Mengembalikan true bila ada obstacle baru / bergeser cukup jauh (perlu replan)
inline bool updateObstacleMap(std::vector<Obs>& map, const std::vector<Detect>& dets,
                              double x, double y, double th) {
  bool changed = false;
  for (size_t i = 0; i < dets.size(); i++) {
    const double Xc = dets[i].X + dets[i].half;   // pusat = muka depan + setengah sisi
    const double gx = x + std::cos(th) * Xc - std::sin(th) * dets[i].Y;
    const double gy = y + std::sin(th) * Xc + std::cos(th) * dets[i].Y;
    int best = -1;
    double bd = 0.25;
    for (size_t j = 0; j < map.size(); j++) {
      const double d = std::hypot(map[j].cx - gx, map[j].cy - gy);
      if (d < bd) { bd = d; best = (int)j; }
    }
    if (best >= 0) {
      const double a = 0.2;
      const double ox = map[best].cx, oy = map[best].cy;
      map[best].cx = (1 - a) * map[best].cx + a * gx;
      map[best].cy = (1 - a) * map[best].cy + a * gy;
      map[best].half = (1 - a) * map[best].half + a * dets[i].half;
      if (std::hypot(map[best].cx - ox, map[best].cy - oy) > 0.02) changed = true;
    } else if (map.size() < 30) {
      Obs o; o.cx = gx; o.cy = gy; o.half = dets[i].half;
      map.push_back(o);
      changed = true;
    }
  }
  return changed;
}


// ---------- Replan dengan histeresis ----------
// Mempertahankan jalur lama bila masih bebas tabrakan dan tidak jauh lebih panjang,
// supaya robot tidak "mengayun" antara sisi kiri dan kanan obstacle.
inline double polyLength(const std::vector<Pt>& p) {
  double L = 0.0;
  for (size_t i = 1; i < p.size(); i++) L += std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
  return L;
}

inline std::vector<Pt> davgReplan(const std::vector<Pt>& prev, const Pt& robot, const Pt& goal,
                                  const std::vector<Obs>& map, bool& ok) {
  std::vector<Pt> fresh = davgPlan(robot, goal, map, ok);
  if (prev.size() < 2 || !ok) return fresh;

  // sisa jalur lama: dari posisi robot ke waypoint lama yang belum terlewati
  size_t seg = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < prev.size(); i++) {
    const double bx = prev[i + 1].x - prev[i].x, by = prev[i + 1].y - prev[i].y;
    const double l2 = bx * bx + by * by;
    if (l2 < 1e-12) continue;
    const double t = clampd(((robot.x - prev[i].x) * bx + (robot.y - prev[i].y) * by) / l2, 0.0, 1.0);
    const double d = std::hypot(robot.x - (prev[i].x + t * bx), robot.y - (prev[i].y + t * by));
    if (d < best) { best = d; seg = i; }
  }
  std::vector<Pt> kept;
  kept.push_back(robot);
  for (size_t i = seg + 1; i < prev.size(); i++) kept.push_back(prev[i]);
  if (kept.size() < 2) return fresh;
  if (std::hypot(kept[1].x - robot.x, kept[1].y - robot.y) < 0.03 && kept.size() > 2)
    kept.erase(kept.begin() + 1);
  if (std::fabs(kept.back().x - goal.x) > 1e-6 || std::fabs(kept.back().y - goal.y) > 1e-6) return fresh;

  std::vector<Rect> test;
  for (size_t i = 0; i < map.size(); i++) test.push_back(inflateObs(map[i], ROBOT_R + VIS_EXTRA));
  for (size_t i = 0; i + 1 < kept.size(); i++)
    if (!visibleSeg(kept[i], kept[i + 1], test)) return fresh;       // jalur lama terblokir
  if (polyLength(kept) > polyLength(fresh) + 0.30) return fresh;     // jalur baru jauh lebih pendek
  return kept;
}

}  // namespace nav
