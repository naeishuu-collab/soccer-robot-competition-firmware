// Bench result reported on 2026-11-27 (TB67H450, 20 kHz, 8-bit, PWM 160): the robot moved correctly in
// these four directions. Rows: front, right, back, left. Columns: M1 right-front, M2 left-front,
// M3 left-rear, M4 right-rear. +1 = PWM on IN1, -1 = PWM on IN2. Do not edit to make a test pass.
#pragma once
#include <Arduino.h>

constexpr int kVerifiedIn1[4] = {10, 12, 6, 8};
constexpr int kVerifiedIn2[4] = {11, 13, 7, 9};
constexpr int kVerifiedSign[4][4] = {
    {-1, 1, 1, -1}, {1, 1, -1, -1}, {1, -1, -1, 1}, {-1, -1, 1, 1}};

inline bool pinsMatchVerified(int row, int duty) {
  for (int m = 0; m < 4; ++m) {
    const bool positive = kVerifiedSign[row][m] > 0;
    if (mockPwm[kVerifiedIn1[m]] != (positive ? duty : 0) || mockPwm[kVerifiedIn2[m]] != (positive ? 0 : duty)) {
      printf("  row %d M%d: IN1(GP%d)=%d IN2(GP%d)=%d\n", row, m + 1, kVerifiedIn1[m], mockPwm[kVerifiedIn1[m]],
             kVerifiedIn2[m], mockPwm[kVerifiedIn2[m]]);
      return false;
    }
  }
  return true;
}
