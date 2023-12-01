/**
 * @file BattSense.h
 * @author Packetworx Inc. (packetworx.com)
 * @brief
 * @version 1.0.0
 * @date 2020-09-30
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef BATTSENSE_H
#define BATTSENSE_H

#include <Arduino.h>

/**
 * @brief Class for the BattSense function
 *
 */
class BattSense {
private:
  /**
   * @brief BattSense pin
   *
   */
  int _pin;

  /**
   * @brief BattSense enable pin
   *
   */
  int _pinEnable;

  /**
   * @brief Number of samples
   *
   */
  int _numSamples;

  /**
   * @brief ADC resolution
   *
   */
  int _resolution;

  /**
   * @brief Max ADC raw value
   *
   */
  int _maxAdcValue;

  /**
   * @brief ADC voltage reference
   *
   */
  float _vRef;

  /**
   * @brief Voltage reduction ratio
   *
   */
  float _vRatio;

  /**
   * @brief Voltage
   *
   */
  float _voltage;

  /**
   * @brief Update battsense reading
   *
   */
  void updateReading();

  /**
   * @brief Get samples
   *
   * @return int Number of samples
   */
  int getSamples();

  /**
   * @brief Convert from raw ADC value to voltage
   *
   * @param raw Raw ADC Value
   * @return float Voltage
   */
  float rawToVoltage(int raw);

public:
  /**
   * @brief Construct a new BattSense object
   *
   * @param pin BattSense pin
   * @param pinEnable BattSense enable pin
   * @param vRef ADC voltage reference
   * @param vRatio Voltage reduction ratio
   * @param numSamples Number of samples
   * @param resolution ADC resolution
   * @param maxAdcValue Max ADC value
   */
  BattSense(int pin, int pinEnable = -1, float vRef = 3.3f, float vRatio = 2.0f,
            int numSamples = 1, int resolution = 12, int maxAdcValue = 4095);

  /**
   * @brief Destroy the Batt Sense object
   *
   */
  ~BattSense();

  /**
   * @brief Set BattSense enable level
   *
   * @param level Pin Level
   * @return true High
   * @return false Low
   */
  bool setEnable(int level);

  /**
   * @brief Get the battery voltage
   *
   * @return float Battery voltage
   */
  float getBatteryVoltage();
};

#endif // !BATTSENSE_H