# STM32_ISR_Servo Library

[![arduino-library-badge](https://www.ardu-badge.com/badge/STM32_ISR_Servo.svg?)](https://www.ardu-badge.com/STM32_ISR_Servo)
[![GitHub release](https://img.shields.io/github/release/khoih-prog/STM32_ISR_Servo.svg)](https://github.com/khoih-prog/STM32_ISR_Servo/releases)
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/khoih-prog/STM32_ISR_Servo/blob/main/LICENSE)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](#Contributing)
[![GitHub issues](https://img.shields.io/github/issues/khoih-prog/STM32_ISR_Servo.svg)](http://github.com/khoih-prog/STM32_ISR_Servo/issues)

---
---

## Table of Contents

* [Changelog](#changelog)
  * [Releases v1.1.0](#releases-v110)
  * [Releases v1.0.0](#releases-v100)

---
---

## Changelog

### Releases v1.1.0

1. Convert to `h-only` library.
2. Optimize library code by using `reference-passing` instead of `value-passing`
3. Improve accuracy by using `float`, instead of `uint32_t` for `position` in degrees
4. Add example [multiFileProject](examples/multiFileProject) to demo for multiple-file project

### Releases v1.0.0

1. Basic 16 ISR-based servo controllers using 1 hardware timer for STM32F/L/H/G/WB/MP1-based board
2. Tested with **STM32L5 (NUCLEO_L552ZE_Q)** and **STM32H7 (NUCLEO_H743ZI2)**


