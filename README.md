# STM32C092KBT6 Project

![MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![Version](https://img.shields.io/badge/version-1.0.0-brightgreen.svg)
![STM32](https://img.shields.io/badge/MCU-STM32C092KBT6-blue.svg)

## 概要

STM32C092KBT6を使用した組み込み制御プロジェクト。

各種通信、タイマー、PWM、LED、モーター制御などの機能を実装する。

## 機能

* GPIO制御
* UART通信
* CAN通信
* タイマー割り込み
* PWM出力
* RGB LED制御
* Flashパラメータ保存

## ピン配置

| Pin  | Function  |
| ---- | --------- |
| PA11 | FDCAN1_RX |
| PB1  | FDCAN1_TX |
| PB7  | USART1_RX |
| PC14 | USART1_TX |
| PA8  | TIM1_CH1  |

## 開発環境

| 項目       | 内容                      |
| -------- | ----------------------- |
| MCU      | STM32C092KBT6           |
| IDE      | STM32CubeIDE for VSCode |
| CubeMX   | STM32CubeMX 6.15.0      |
| Firmware | STM32CubeC0             |
| Language | C                       |

## 実装予定機能

* [ ] CAN通信
* [ ] モーター制御
* [ ] 加減速制御
* [ ] Flashパラメータ管理
* [ ] RGB LED制御
* [ ] エラー処理
* [ ] ウォッチドッグ
