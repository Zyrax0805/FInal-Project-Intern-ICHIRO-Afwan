#pragma once
// ============================================================
//  config.hpp : semua parameter yang mungkin perlu Anda ubah
// ============================================================
#include <cmath>

namespace nav {

// ---------- KAMERA (kalibrasi di MODE 0) ----------
static const double CAM_HEIGHT = 0.45;  // m  : tinggi lensa dari lantai
static const double CAM_PITCH  = 0.30;  // rad: kemiringan kamera ke BAWAH dari horizontal
static const double CAM_X_OFF  = 0.05;  // m  : jarak lensa di depan titik pusat badan
static const double HEAD_TILT  = 1;   // rad: perintah motor "Head" (kepala dikunci di sini)

// ---------- KALIBRASI KAMERA OTOMATIS (opsional, sangat disarankan) ----------
// Jalankan MODE 0, taruh SATU obstacle merah tepat di depan robot pada beberapa jarak,
// salin nilai v= dari log, dan isi jarak sebenarnya (m) dari titik tengah robot ke MUKA depan obstacle.
// Bila CALIB_N >= 2, program mencari CAM_HEIGHT dan CAM_PITCH sendiri saat start.
static const int    CALIB_N    = 4;
static const double CALIB_V[8] = {120, 110, 95, 89, 84, 82, 79, 76};   // contoh: {88, 74, 66, 60}
static const double CALIB_D[8] = {0.5, 1, 1.3, 1.5, 1.7, 1.8, 2, 2.2};   // contoh: {0.40, 0.70, 1.00, 1.40}

// ---------- WARNA OBSTACLE (HSV, skala OpenCV: H 0-180) : default MERAH ----------
static const int HUE1_LO = 0,   HUE1_HI = 10;
static const int HUE2_LO = 170, HUE2_HI = 180;
static const int SAT_MIN = 120, VAL_MIN = 70;

// ---------- GERAK (kalibrasi di MODE 1 dan 2) ----------
static const double KV        = 0.09;   // m/s per 1.0 xAmplitude
static const double KW        = 1;   // rad/s per 1.0 aAmplitude
static const double A_SIGN    = 1.0;    // +1 bila aAmplitude positif = belok KIRI (CCW); -1 bila kanan
static const double X_AMP_MAX = 0.6;
static const double A_AMP_MAX = 0.5;
static const double V_MAX     = KV * X_AMP_MAX;
static const double W_MAX     = KW * A_AMP_MAX;
static const double V_REF     = 0.7 * V_MAX;

// ---------- TUGAS (frame awal: titik robot saat start = (0,0), hadap +x) ----------
static const double GOAL_X      = 1.0;   // m : jarak ke garis finish searah hadap awal
static const double GOAL_Y      = 0.0;
static const double GOAL_TOL    = 0.10;  // m
static const double ARENA_Y_LIM = 2;  // m : batas |y| yang boleh dilewati

// ---------- KEAMANAN & OBSTACLE ----------
static const double ROBOT_R       = 0.18;  // m : radius aman robot
static const double SAFETY_MARGIN = 0.07;  // m : tambahan margin DAVG (posisi sudut node)
static const double VIS_EXTRA     = 0.03;  // m : menutup celah sempit antar obstacle (< 2*VIS_EXTRA)
static const double OBS_HALF_MIN  = 0.05;  // m : setengah sisi minimum obstacle
static const double OBS_HALF_MAX  = 0.20;
static const double DET_MIN_DIST  = 0.10;  // m : jarak minimum yang dipakai (lebih dekat dianggap segini)
static const double DET_MAX_DIST  = 3.00;  // m : deteksi lebih jauh dari ini diabaikan
static const double TURN_GATE     = 0.12;  // rad/s: peta tidak diperbarui saat berputar lebih cepat dari ini

// ---------- LMPC ----------
static const int    MPC_N  = 10;     // horizon (langkah)
static const double MPC_DT = 0.4;    // detik per langkah prediksi
static const double Q_X = 40.0, Q_Y = 60.0, Q_TH = 6.0;  // bobot error posisi/heading
static const double QF_SCALE = 2.0;                       // pengali bobot langkah terakhir
static const double R_V = 2.0,  R_W = 0.5;                // bobot deviasi input dari referensi
static const double S_V = 4.0,  S_W = 1.0;                // bobot perubahan input (kehalusan)
static const int    QP_ITERS = 200;

}  // namespace nav
