# BattSense

## Version

Version 1.0.0

## Information

This library is for using and ADC pin of the MCU as a battery voltage level reader in conjuction with a circuit to reduce the voltage to a certain ratio.

## Usage

### Constructor

```cpp
BattSense(
  int pin,
  int pinEnable = -1,
  float vRef = 3.3f,
  float vRatio = 2.0f,
  int numSamples = 1,
  int resolution = 12,
  int maxAdcValue = 4095
);
```

Parameters:

- `int` **pin** = BattSense ADC pin
- `int` **pinEnable** = BattSense enable pin
- `float` **vRef** = ADC voltage reference
- `float` **vRatio** = Voltage Reduction Ratio
- `int` **numSamples** = Number of samples to acquire
- `int` **resolution** = ADC bit resolution
- `int` **maxAdcValue** = Maximum raw ADC value

### Functions

#### Set Enable Level

```cpp
bool setEnable(int level);
```

Parameters:

- `int` **level** = Digital voltage level to set on the BattSense enable pin

Returns:

- `bool` **true** = If BattSense enable pin is declared
- `bool` **false** = If BattSense enable pin is not declared

#### Get Battery Voltage

```cpp
float getBatteryVoltage();
```

Returns:

- The **Battery Voltage** read by the BattSense pin in `float` data type.

---

> Last Modifed on `2020-09-30`
