// ============================================================
//  main.cpp : Robotis OP2 - obstacle avoidance dengan DAVG + LMPC   (v2)
//  Sensor obstacle: HANYA kamera (segmentasi warna).
//  Pose robot: dead reckoning dari perintah gait (tanpa GPS/supervisor).
//
//  MODE 0 : robot diam, uji persepsi + data kalibrasi kamera
//  MODE 1 : kalibrasi kecepatan maju  (KV)
//  MODE 2 : kalibrasi kecepatan putar (KW dan A_SIGN)
//  MODE 3 : simulasi penuh start -> finish   (DEFAULT)
// ============================================================
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "config.hpp"
#include "camcal.hpp"
#include "davg.hpp"
#include "lmpc.hpp"

using namespace managers;
using namespace nav;

// ------------------- PENGATURAN YANG SERING DIUBAH -------------------
static const int    MODE        = 3;     // 0, 1, 2, atau 3
static const double WARMUP_SEC  = 1.5;   // diam dulu agar kamera memetakan obstacle awal
static const double CAL_SEC     = 10.0;  // lama berjalan pada mode kalibrasi
static const double CAL_X_AMP   = 0.5;   // xAmplitude saat kalibrasi lurus
static const double CAL_A_AMP   = 0.3;   // aAmplitude saat kalibrasi putar
static const int    PRINT_EVERY = 30;    // cetak status tiap N langkah simulasi
// ----------------------------------------------------------------------

static double g_camH = CAM_HEIGHT;       // diganti hasil kalibrasi otomatis bila tersedia
static double g_camPitch = CAM_PITCH;

// Kepala dikunci SETELAH gm->step(): perintah terakhir sebelum robot->step() yang berlaku,
// sehingga tidak ditimpa gait manager.
static void holdHead(webots::Motor* neck, webots::Motor* head) {
  if (neck) neck->setPosition(0.0);
  if (head) head->setPosition(HEAD_TILT);
}

struct Diag { int contours, small_, clipped, above, far_, accepted; };

// Deteksi obstacle -> posisi di frame robot (X depan, Y kiri) lewat geometri lantai datar
static std::vector<Detect> perceive(const cv::Mat& bgr, double f, cv::Mat& dbg, Diag& dg) {
  const int W = bgr.cols, H = bgr.rows;
  dg.contours = dg.small_ = dg.clipped = dg.above = dg.far_ = dg.accepted = 0;

  cv::Mat hsv, m1, m2, mask;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(HUE1_LO, SAT_MIN, VAL_MIN), cv::Scalar(HUE1_HI, 255, 255), m1);
  cv::inRange(hsv, cv::Scalar(HUE2_LO, SAT_MIN, VAL_MIN), cv::Scalar(HUE2_HI, 255, 255), m2);
  mask = m1 | m2;
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  dbg = bgr.clone();
  std::vector<Detect> out;
  for (size_t i = 0; i < contours.size(); i++) {
    dg.contours++;
    if (cv::contourArea(contours[i]) < 25.0) { dg.small_++; continue; }
    const cv::Rect r = cv::boundingRect(contours[i]);
    const double u = r.x + r.width * 0.5;
    double v = r.y + r.height;                        // dasar obstacle (menyentuh lantai)
    const bool clippedBottom = (r.y + r.height >= H - 1);
    if (clippedBottom) { v = H; dg.clipped++; }       // terpotong: obstacle SETIDAKNYA sedekat titik ini

    double X, Y;
    if (!pixelToFloor(u, v, W, H, f, g_camH, g_camPitch, CAM_X_OFF, X, Y)) {
      dg.above++;
      cv::rectangle(dbg, r, cv::Scalar(0, 255, 255), 1);
      continue;
    }
    if (X > DET_MAX_DIST) {
      dg.far_++;
      cv::rectangle(dbg, r, cv::Scalar(0, 255, 255), 1);
      continue;
    }
    if (X < DET_MIN_DIST) X = DET_MIN_DIST;

    const double depth = std::max(0.05, (X - CAM_X_OFF) / std::max(0.2, std::cos(g_camPitch)));
    Detect d;
    d.X = X; d.Y = Y; d.u = u; d.v = v;
    d.half = clampd(0.5 * (r.width / f) * depth, OBS_HALF_MIN, OBS_HALF_MAX);
    out.push_back(d);
    dg.accepted++;

    cv::rectangle(dbg, r, cv::Scalar(0, 255, 0), 1);
    cv::circle(dbg, cv::Point((int)u, (int)v), 3, cv::Scalar(255, 0, 0), -1);
    char txt[64];
    std::snprintf(txt, sizeof(txt), "X%.2f Y%.2f", d.X, d.Y);
    cv::putText(dbg, txt, cv::Point(r.x, std::max(10, r.y - 3)), cv::FONT_HERSHEY_SIMPLEX, 0.35,
                cv::Scalar(255, 255, 255), 1);
  }
  return out;
}

int main() {
  std::printf("==== LMPC + DAVG controller v2 | MODE=%d ====\n", MODE);
  std::fflush(stdout);

  webots::Robot* robot = new webots::Robot();
  const int ts = (int)robot->getBasicTimeStep();
  const double dt = ts / 1000.0;

  RobotisOp2MotionManager* mm = new RobotisOp2MotionManager(robot);
  RobotisOp2GaitManager* gm = new RobotisOp2GaitManager(robot, "config.ini");

  webots::Camera* cam = robot->getCamera("Camera");
  if (cam == NULL) {
    std::cerr << "ERROR: Camera not found!" << std::endl;
    delete robot;
    return 1;
  }
  cam->enable(ts);
  const int W = cam->getWidth(), H = cam->getHeight();
  const double f = (W / 2.0) / std::tan(cam->getFov() / 2.0);

  webots::Motor* neck = robot->getMotor("Neck");
  webots::Motor* head = robot->getMotor("Head");
  if (head)
    std::printf("Head motor range: [%.3f, %.3f] rad. HEAD_TILT = %.3f\n",
                head->getMinPosition(), head->getMaxPosition(), HEAD_TILT);
  std::printf("Camera %dx%d FOV=%.3f f=%.1fpx | timestep=%dms\n", W, H, cam->getFov(), f, ts);

  // Kalibrasi kamera otomatis dari data yang Anda isi di config.hpp
  if (CALIB_N >= 2) {
    CamFit ft = fitCamera(CALIB_V, CALIB_D, CALIB_N, H, f, CAM_X_OFF, CAM_HEIGHT, CAM_PITCH);
    g_camH = ft.h;
    g_camPitch = ft.pitch;
    std::printf("Kalibrasi kamera: tinggi=%.3f m, pitch=%.3f rad, galat relatif RMS=%.3f %s\n",
                ft.h, ft.pitch, ft.rms, ft.rms > 0.15 ? "(BESAR: periksa data CALIB_V/D)" : "(baik)");
  } else {
    std::printf("Kalibrasi kamera: memakai default tinggi=%.2f pitch=%.2f (isi CALIB_* untuk akurasi)\n",
                g_camH, g_camPitch);
  }
  std::printf("Tujuan: GOAL=(%.2f, %.2f) m dari titik start | batas |y|<=%.2f m\n", GOAL_X, GOAL_Y,
              ARENA_Y_LIM);
  std::fflush(stdout);

  // Pose awal
  robot->step(ts);
  mm->playPage(9);
  for (int i = 0; i < 200 / ts + 1; i++) robot->step(ts);

  bool gaitOn = false, finished = false, calDone = false, pendingReplan = true;
  double x = 0.0, y = 0.0, th = 0.0;          // pose estimasi (frame awal)
  double uvPrev = 0.0, uwPrev = 0.0;          // kecepatan terakhir yang diperintahkan
  std::vector<Obs> map;
  std::vector<Pt> path;
  long lastPlan = -100000;
  long k = 0;

  while (robot->step(ts) != -1) {
    const double t = k * dt;
    double xAmp = 0.0, aAmp = 0.0;

    // ---------------- Persepsi ----------------
    // (mode kalibrasi gerak 1/2 tidak memproses gambar, agar sama persis dengan program uji jalan lurus)
    std::vector<Detect> dets;
    cv::Mat dbg;
    Diag dg = {0, 0, 0, 0, 0, 0};
    const unsigned char* img = (MODE == 0 || MODE == 3) ? cam->getImage() : NULL;
    if (img) {
      cv::Mat bgra(H, W, CV_8UC4, (void*)img);
      cv::Mat bgr;
      cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
      dets = perceive(bgr, f, dbg, dg);
    }

    // ---------------- MODE 0: uji persepsi ----------------
    if (MODE == 0) {
      if (k % PRINT_EVERY == 0) {
        std::printf("t=%.1fs kontur=%d (kecil=%d terpotong=%d diatas_horizon=%d terlalu_jauh=%d) diterima=%d",
                    t, dg.contours, dg.small_, dg.clipped, dg.above, dg.far_, dg.accepted);
        for (size_t i = 0; i < dets.size(); i++)
          std::printf("  [v=%.0f u=%.0f -> X=%.2f Y=%.2f half=%.3f]", dets[i].v, dets[i].u, dets[i].X,
                      dets[i].Y, dets[i].half);
        std::printf("\n");
        if (!dbg.empty()) cv::imwrite("debug.png", dbg);
      }
      holdHead(neck, head);
      k++;
      continue;
    }

    // ---------------- MODE 1 / 2: kalibrasi gerak ----------------
    if (MODE == 1 || MODE == 2) {
      if (!gaitOn && t >= 0.3) {
        gm->setBalanceEnable(true);
        gm->start();
        gaitOn = true;
        std::printf(">>> KALIBRASI MULAI (%.0f s). Posisi/rotasi awal = nilai di world sebelum Run.\n", CAL_SEC);
      }
      if (gaitOn) {
        const double tw = t - 0.3;
        if (tw < CAL_SEC) {
          xAmp = (MODE == 1) ? CAL_X_AMP : 0.0;
          aAmp = (MODE == 2) ? CAL_A_AMP : 0.0;
        } else if (!calDone) {
          calDone = true;
          std::printf(">>> KALIBRASI SELESAI. Baca posisi/rotasi akhir robot di Scene Tree.\n");
        }
        gm->setXAmplitude(xAmp);
        gm->setAAmplitude(aAmp);
        gm->step(ts);
        if (k % PRINT_EVERY == 0)
          std::printf("t=%.1fs  perintah gait: xAmp=%.2f aAmp=%.2f\n", t, xAmp, aAmp);
      }
      holdHead(neck, head);
      k++;
      continue;
    }

    // ---------------- MODE 3: DAVG + LMPC ----------------
    // Peta hanya diperbarui saat robot tidak sedang berputar cepat (estimasi heading lebih andal)
    if (std::fabs(uwPrev) < TURN_GATE && updateObstacleMap(map, dets, x, y, th)) pendingReplan = true;

    if (t >= WARMUP_SEC && !gaitOn) {
      gm->setBalanceEnable(true);
      gm->start();
      gaitOn = true;
      std::printf(">>> Gait dimulai pada t=%.1f s\n", t);
    }

    if (gaitOn && !finished) {
      const Pt goal = {GOAL_X, GOAL_Y};
      if (std::hypot(goal.x - x, goal.y - y) < GOAL_TOL) {
        finished = true;
        std::printf(">>> FINISH tercapai (estimasi). Pose est = (%.2f, %.2f), t = %.1f s\n", x, y, t);
      } else {
        // DAVG: bangun ulang graf visibilitas bila peta berubah / berkala
        if (path.empty() || (pendingReplan && k - lastPlan > 10) || k - lastPlan > 60) {
          bool ok = true;
          path = davgReplan(path, Pt{x, y}, goal, map, ok);
          lastPlan = k;
          pendingReplan = false;
          if (!ok) std::printf("WARNING: DAVG tidak menemukan jalur bebas, memakai garis lurus.\n");
        }
        // LMPC: ikuti jalur DAVG
        const std::vector<Ref> ref = sampleReference(path, Pt{x, y}, goal);
        const double pose[3] = {x, y, th};
        double v = 0.0, w = 0.0;
        if (lmpcSolve(pose, ref, uvPrev, uwPrev, v, w)) {
          xAmp = clampd(v / KV, 0.0, X_AMP_MAX);
          aAmp = clampd(A_SIGN * w / KW, -A_AMP_MAX, A_AMP_MAX);
        }
      }
    }

    if (gaitOn) {
      gm->setXAmplitude(xAmp);
      gm->setAAmplitude(aAmp);
      gm->step(ts);
    }
    holdHead(neck, head);

    // Dead reckoning dari perintah yang benar-benar dikirim
    uvPrev = KV * xAmp;
    uwPrev = A_SIGN * KW * aAmp;
    x += uvPrev * std::cos(th) * dt;
    y += uvPrev * std::sin(th) * dt;
    th = wrapPi(th + uwPrev * dt);

    if (k % PRINT_EVERY == 0) {
      std::printf("t=%6.1fs pose=(%.2f,%.2f,%.2f) peta=%d jalur=%d v=%.3f w=%.3f xAmp=%.2f aAmp=%.2f | kontur=%d diterima=%d terpotong=%d\n",
                  t, x, y, th, (int)map.size(), (int)path.size(), uvPrev, uwPrev, xAmp, aAmp,
                  dg.contours, dg.accepted, dg.clipped);
      if (!dbg.empty() && k % (PRINT_EVERY * 4) == 0) cv::imwrite("debug.png", dbg);
    }
    k++;
  }

  delete robot;
  return 0;
}
