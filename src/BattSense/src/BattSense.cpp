/**
 * @file BattSense.cpp
 * @author Packetworx Inc. (packetworx.com)
 * @brief
 * @version 1.0.0
 * @date 2020-09-30
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "./BattSense.h"

BattSense::BattSense(int pin, int pinEnable, float vRef, float vRatio,
                     int numSamples, int resolution, int maxAdcValue) {
  _pin = pin;
  _pinEnable = pinEnable;
  _vRef = vRef;
  _vRatio = vRatio;
  _numSamples = numSamples;
  _resolution = resolution;
  _maxAdcValue = maxAdcValue;
}

BattSense::~BattSense() {}

void BattSense::updateReading() {
  analogReadResolution(_resolution);
  _voltage = rawToVoltage(getSamples());
}

int BattSense::getSamples() {
  int average = 0;
  for (int i = 0; i < _numSamples; i++) {
    average += analogRead(_pin);
  }
  return (average / _numSamples);
}

float BattSense::rawToVoltage(int raw) {
  float voltage = (float)raw * _vRef / (float)_maxAdcValue;
  return (voltage);
}

bool BattSense::setEnable(int level) {
  if (_pinEnable != (-1)) {
    pinMode(_pinEnable, OUTPUT);
    digitalWrite(_pinEnable, level);
    return (true);
  }
  return (false);
}

float BattSense::getBatteryVoltage() {
  updateReading();
  return (_voltage * _vRatio);
}