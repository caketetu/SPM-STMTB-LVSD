/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SPM_Modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//ピン定義
#define TQ_PIN GPIO_PIN_8     // トルクアップ信号ピン
#define TQ_PORT GPIOB         // トルクアップ信号ポート
#define EN_PIN GPIO_PIN_7     // イネーブル信号ピン
#define EN_PORT GPIOB         // イネーブル信号ポート
#define M2_PIN GPIO_PIN_6     // モータ2信号ピン
#define M2_PORT GPIOB         // モータ2信号ポート
#define M1_PIN GPIO_PIN_3     // モータ1信号ピン
#define M1_PORT GPIOB         // モータ1信号ポート
#define DIR_PIN GPIO_PIN_2    // 方向信号ピン
#define DIR_PORT GPIOB        // 方向信号ポート
#define STBY_PIN GPIO_PIN_1   // スタンバイ信号ピン
#define STBY_PORT GPIOB       // スタンバイ信号ポート

#define HOME_PIN GPIO_PIN_9   // HOME(IO0)信号ピン
#define HOME_PORT GPIOB       // HOME信号(IO0)ポート
#define H_LIMIT_PIN GPIO_PIN_3   // H_LIMIT(IO1)信号ピン
#define H_LIMIT_PORT GPIOA       // H_LIMIT(IO1)信号ポート
#define L_LIMIT_PIN GPIO_PIN_4   // L_LIMIT(IO2)信号ピン
#define L_LIMIT_PORT GPIOA       // L_LIMIT(IO2)信号ポート
#define EN_2_PIN GPIO_PIN_5   // enable信号ピン
#define EN_2_PORT GPIOB       // enable信号ポート

//ステータス定義ビットマップ
//0 動作中
//1,2,3 動作モード rw_modeの下位3ビットと同じ
//4,5 動作ステータス 0停止　1加速　2減速　3定速
//6 インポジション
//7 na
//8 enable ピン信号
//9 tq ピン信号
//10 enable ピン信号
//11 IO0ピン信号
//12 IO1ピン信号
//13 IO2ピン信号
//14,15 na

#define STATUS_ACTIVE       (1U << 0)
#define STATUS_MODE_MASK    (0x7U << 1)
#define STATUS_MOTION_MASK  (0x3U << 4)
#define STATUS_IN_POSITION  (1U << 6)
#define STATUS_ENABLE       (1U << 8)
#define STATUS_TQ           (1U << 9)
#define STATUS_ENABLE_2     (1U << 10)
#define STATUS_IO0          (1U << 11)
#define STATUS_IO1          (1U << 12)
#define STATUS_IO2          (1U << 13)

//エラー ビットマップ
//0 パルス追従エラー
//1 指令値エラー
//2 上限リミット
//3 下限リミット
//4 非常停止
//5-15 na

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// グローバル変数
//_は内部変数、roは読み取り専用変数(input_regs)
//rwは読み書き可能な変数(holding_regs)
//デバイス定義
int16_t ro_model_no = 52;   // モデル番号
int16_t ro_version = 1;     // バージョン番号
int16_t ro_sub_version = 0; // サブバージョン番号

//内部変数
//シリアルデータ
uint8_t _recv_data;
uint8_t _rx_buf[256];
uint8_t _tx_buf[256];
int _rx_len=0;
//アプリケーション変数
int16_t rw_dev_id=51; //デバイスID
//ステータス関連
volatile int16_t ro_status = 0; //ステータス
volatile int16_t rw_error = 0; // エラーステータス
//動作モード関連
volatile int16_t rw_mode = 0;  //動作モード　0:絶対位置制御, 1:速度制御　(最上位が動作指令ON)
volatile int16_t rw_io0_mode = 0; // IO0モード 0:無効 1:下限リミット(ONで有効) 2:下限リミット(OFFで有効)
volatile int16_t rw_io1_mode = 0; // IO1モード 0:無効 1:HOME(ONで有効) 2:HOME(OFFで有効)
volatile int16_t rw_io2_mode = 0; // IO2モード 0:無効 1:上限リミット(ONで有効) 2:上限リミット(OFFで有効)
//ステップ数
volatile int32_t _pulse_count = 0; //現在ステップ数
volatile int32_t _pulse_target = 0; //目標ステップ数
volatile int32_t _pulse_start = 0; //動作開始時のステップ数
volatile int32_t _pulse_direction = 1; //正転:+1、反転:-1
volatile int16_t ro_pulse_acc = 0; //加速ステップ数 (Acceleration steps) #自動計算
volatile int16_t ro_pulse_dec = 0; //減速ステップ数 (Deceleration steps) #自動計算
//ステップ数(表示用) 表示用は必ずint16_tで定義すること
volatile int16_t rw_pulse_count_h = 0; //現在ステップ数の上位16ビット(表示用)
volatile int16_t rw_pulse_count_l = 0; //現在ステップ数の下位16ビット(表示用)
volatile int16_t rw_pulse_target_h = 0; //目標ステップ数の上位16ビット(表示用)
volatile int16_t rw_pulse_target_l = 0; //目標ステップ数の下位16ビット(表示用)
//速度関連
volatile int32_t _next_period_us = 0; //次の周期時間 (マイクロ秒)
volatile int32_t _FL_speed = 0; //初速 period
volatile int32_t _FH_speed = 0; //最高速 period
volatile int32_t _FH_direction = 1; //最高速の方向
volatile int32_t _position_speed_hz = 0; //位置制御の現在速度Hz
//速度関連(表示用)
volatile int16_t ro_next_period_hz = 0; //次の周期周波数 (Hz) #表示用
volatile int16_t rw_FL_speed_hz = 0; //初速 (表示用)
volatile int16_t rw_FH_speed_hz = 0; //最高速 (表示用)
volatile int16_t rw_ACC_ratio = 0; //加速レート (表示用)
//その他

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//絶対値を返す関数32ビット版
int32_t abs32(int32_t x)
{
    return (x < 0) ? -x : x;
}
//絶対値を返す関数16ビット版
int16_t abs16(int16_t x)
{
    return (x < 0) ? -x : x;
}

// 上位16ビットと下位16ビットから32ビットのステップ数を結合する関数
int32_t combine_words(int16_t high_word, int16_t low_word)
{
  if (high_word == 0 && low_word < 0)
  {
    return low_word;
  }
  return ((int32_t)high_word << 16) | (uint16_t)low_word;
}

//正負か0を返す関数16ビット版
int16_t sign16(int16_t x)
{
    return (x > 0) - (x < 0);
}

//正負か0を返す関数32ビット版
int32_t sign32(int32_t x)
{
    return (x > 0) - (x < 0);
}

//周期[us]から周波数[Hz]を計算する関数
int32_t period_to_frequency(int32_t period_us)
{
  if (period_us == 0)
        return 0;
  return (period_us < 0)
     ? -(1000000 / abs32(period_us))
     : (1000000 / period_us);
}
//周波数[Hz]から周期[us]を計算する関数
int32_t frequency_to_period(int32_t frequency_hz)
{
    if (frequency_hz <= 0)
        return 0;
    return 1000000 / frequency_hz;
}

void UpdateStatusInputs(void)
{
  ro_status &= (STATUS_ACTIVE | STATUS_MODE_MASK | STATUS_MOTION_MASK
        | STATUS_IN_POSITION);
  ro_status |= ((uint16_t)(rw_mode & 0x7) << 1);

  if (__HAL_TIM_GET_COUNTER(&htim1) != 0U)
  {
    ro_status |= STATUS_ACTIVE;
  }
  if (HAL_GPIO_ReadPin(EN_PORT, EN_PIN) == GPIO_PIN_SET)
  {
    ro_status |= STATUS_ENABLE | STATUS_ENABLE_2;
  }
  if (HAL_GPIO_ReadPin(TQ_PORT, TQ_PIN) == GPIO_PIN_SET)
  {
    ro_status |= STATUS_TQ;
  }
  if (HAL_GPIO_ReadPin(HOME_PORT, HOME_PIN) == GPIO_PIN_SET)
  {
    ro_status |= STATUS_IO0;
  }
  if (HAL_GPIO_ReadPin(H_LIMIT_PORT, H_LIMIT_PIN) == GPIO_PIN_SET)
  {
    ro_status |= STATUS_IO1;
  }
  if (HAL_GPIO_ReadPin(L_LIMIT_PORT, L_LIMIT_PIN) == GPIO_PIN_SET)
  {
    ro_status |= STATUS_IO2;
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //各レジスタ初期化
  SPM_ModbusInit();
  //Input (Read-Only) Registers
  //アプリケーション変数のうちのro(input_regs)+rw(holding_regs)
  p_input_regs[0] = &ro_model_no;   //モデルナンバー
  p_input_regs[1] = &ro_version; //バージョン
  p_input_regs[2] = &ro_sub_version; //サブバージョン
  p_input_regs[3] = &rw_dev_id;
  p_input_regs[4] = &ro_status; //ステータス
  p_input_regs[5] = &rw_error; //エラーコード
  p_input_regs[6] = &rw_mode; //モード
  p_input_regs[7] = &rw_io0_mode; //IO0モード
  p_input_regs[8] = &rw_io1_mode; //IO1モード
  p_input_regs[9] = &rw_io2_mode; //IO2モード
  p_input_regs[10] = &ro_pulse_acc; //加速レート (表示用)
  p_input_regs[11] = &ro_pulse_dec; //減速レート (表示用)
  p_input_regs[12] = &rw_pulse_count_h; //パルスカウント上位
  p_input_regs[13] = &rw_pulse_count_l; //パルスカウント下位
  p_input_regs[14] = &rw_pulse_target_h; //パルスターゲット上位
  p_input_regs[15] = &rw_pulse_target_l; //パルスターゲット下位
  p_input_regs[16] = &ro_next_period_hz; //次の周期周波数 (表示用)
  p_input_regs[17] = &rw_FL_speed_hz; //初速 (表示用)
  p_input_regs[18] = &rw_FH_speed_hz; //最高速 (表示用)
  p_input_regs[19] = &rw_ACC_ratio; //加速比 (表示用)

  //Holding Registers (Read/Write)
  //アプリケーション変数のうちのrw(holding_regs)
  p_holding_regs[0] = &rw_dev_id;
  //rwモード関連
  p_holding_regs[1] = &rw_mode;
  p_holding_regs[2] = &rw_io0_mode;
  p_holding_regs[3] = &rw_io1_mode;
  p_holding_regs[4] = &rw_io2_mode;
  p_holding_regs[5] = &rw_error;
  //rwパルス関連
  p_holding_regs[6] = &rw_pulse_count_h;
  p_holding_regs[7] = &rw_pulse_count_l;
  p_holding_regs[8] = &rw_pulse_target_h;
  p_holding_regs[9] = &rw_pulse_target_l;
  //rw速度関連
  p_holding_regs[10] = &rw_FL_speed_hz;
  p_holding_regs[11] = &rw_FH_speed_hz;
  p_holding_regs[12] = &rw_ACC_ratio;

  //Cyclic Function0登録 位置制御
  cycfunc0.rx_len = 5;
  cycfunc0.rx_adr[0] = &ro_status;
  cycfunc0.rx_adr[1] = &rw_error;
  cycfunc0.rx_adr[2] = &rw_pulse_count_h;
  cycfunc0.rx_adr[3] = &rw_pulse_count_l;
  cycfunc0.rx_adr[4] = &ro_next_period_hz;
  cycfunc0.tx_len = 4;
  cycfunc0.tx_adr[0] = &rw_mode;
  cycfunc0.tx_adr[1] = &rw_pulse_target_h;
  cycfunc0.tx_adr[2] = &rw_pulse_target_l;
  cycfunc0.tx_adr[3] = &rw_FH_speed_hz;

  ParamLoad();
  SPM_ModbusSetAddress(rw_dev_id);
  HAL_UART_Receive_IT(&huart2, &_recv_data, 1);
  HAL_TIM_Base_Start_IT(&htim14); // Start TIM14 for periodic interrupts
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // LED ON
  
  //FDCAN1 filter configuration
  // FDCAN_FilterTypeDef sFilterConfig;
  // sFilterConfig.IdType = FDCAN_STANDARD_ID;
  // sFilterConfig.FilterIndex = 0;
  // sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // sFilterConfig.FilterID1 = 0x123; // Standard ID is configured as-is (no manual bit shift)
  // sFilterConfig.FilterID2 = 0x7FF; // Filter mask (accept all IDs)
  // if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Filter configuration Error
  //   Error_Handler();
  // }else{
    
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  // // Start FDCAN module
  // if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Start Error
  //   Error_Handler();
  // }else{
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  //ピン設定
  HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(CK_PORT, CK_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TQ_PORT, TQ_PIN, GPIO_PIN_RESET);

  //励磁方式設定　M1,M2
  HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_SET);

  //Enable
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // //FDCAN送信
    // FDCAN_TxHeaderTypeDef TxHeader = {0};
    // uint8_t CanTxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, counter++}; // 送信データ
    // TxHeader.Identifier = 0x143; // 送信するID
    // TxHeader.IdType = FDCAN_STANDARD_ID;
    // TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    // TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    // TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    // TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    // TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    // TxHeader.MessageMarker = 0;
    // if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) != HAL_OK)
    // {
    //   // Transmission request Error
    //   TxDataLength = sprintf((char*)TxData, "FDCAN Transmission Error\r\n");
    //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
    //   Error_Handler();
    // }else{
    //   TxDataLength = sprintf((char*)TxData, "FDCAN Transmission Success\r\n");
    //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
    // }

    //シリアル通信処理
		if(_rx_len>0){
			HAL_Delay(10);
			int16_t tx_len = SPM_ModbusTask(_rx_buf, _rx_len, _tx_buf);
			HAL_UART_Transmit(&huart2, _tx_buf, tx_len, 100);
			_rx_len=0;
      //パルスカウントをリセット
      _pulse_count = combine_words(rw_pulse_count_h,
                     rw_pulse_count_l);
      _pulse_target = combine_words(rw_pulse_target_h,
                     rw_pulse_target_l);
		}

    //ステータスアップデート
    UpdateStatusInputs();

    // rw_modeのbit7が動作指令。下位7bitで制御モードを選択する。
    // 0:位置制御、1:速度制御。指令は1回処理したらbit7をクリアする。
    if(rw_mode & 0x80){
      int16_t requested_mode = rw_mode & 0x7F;
      rw_mode = requested_mode;
      switch(requested_mode){
          case 0:
              //速度と加速レートを周期に変換
              //位置制御の時は正の速度のみ使用する
              rw_FL_speed_hz = abs16(rw_FL_speed_hz);
              _FL_speed = frequency_to_period(rw_FL_speed_hz);
              _FH_speed = frequency_to_period(abs16(rw_FH_speed_hz));
              if(_FH_speed == 0){rw_error = 1; break;}
              rw_ACC_ratio = abs16(rw_ACC_ratio);

              _pulse_start = _pulse_count;  // 現在のパルスカウントを開始位置として記録
              ro_status |= STATUS_ACTIVE | (1U << 4); // 制御モードをアクティブに設定

              // パルス方向を決定
              if (_pulse_count < _pulse_target) _pulse_direction = 1;
              else if (_pulse_count > _pulse_target) _pulse_direction = -1;
              else break;

              _FH_direction = _pulse_direction;   // FHの回転方向を設定
              _position_speed_hz = rw_FL_speed_hz;  // 位置制御時の目標速度
              // 加速パルス数を計算
              ro_pulse_acc = (abs16(rw_FH_speed_hz) - rw_FL_speed_hz)
                           / rw_ACC_ratio;
              // 加速位置が1未満にならないように調整
              if (ro_pulse_acc < 1) ro_pulse_acc = 1;
              // △: 加速位置が全移動距離の半分を超えないように調整
              if (ro_pulse_acc * 2 > abs32(_pulse_target - _pulse_start))
                  ro_pulse_acc = abs32(_pulse_target - _pulse_start) / 2;
              //減速パルス数を計算
              ro_pulse_dec = ro_pulse_acc;
              //初期速度を設定
              _next_period_us = _pulse_direction * _FL_speed;
              //回転方向を設定
              HAL_GPIO_WritePin(
                DIR_PORT,
                DIR_PIN,
                (_pulse_direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
              // タイマーが停止している場合に初期設定を行う
              if ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U)
              {
                __HAL_TIM_SET_AUTORELOAD(&htim1, _FL_speed - 1);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                                      _FL_speed / 2);
                __HAL_TIM_SET_COUNTER(&htim1, 0);
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
                __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
              }
              break;
          case 1:
              // 速度制御: FH速度の符号を回転方向、絶対値を目標速度として使用する。
              rw_FL_speed_hz = abs16(rw_FL_speed_hz);
              rw_ACC_ratio = abs16(rw_ACC_ratio);
              _FL_speed = frequency_to_period(rw_FL_speed_hz); // 初速(periodに変換)
              _FL_speed = abs32(_FL_speed);
              if (rw_FH_speed_hz == 0)
              {
                // FH速度が0の場合は回転を停止
                _FH_speed = 0;
              }
              else
              {
                // FH速度が0でない場合は方向付き最高速を設定
                _FH_speed = frequency_to_period(abs16(rw_FH_speed_hz))
                          * sign16(rw_FH_speed_hz); // 方向付き最高速period
              }

              // タイマーが停止している場合に初期設定を行う
              if (_FH_speed != 0
                  && (htim1.Instance->CR1 & TIM_CR1_CEN) == 0U)
              {
                ro_status |= STATUS_ACTIVE | (1U << 4);
                _next_period_us = _FL_speed * sign32(_FH_speed);
                __HAL_TIM_SET_AUTORELOAD(&htim1, abs32(_next_period_us) - 1);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,abs32(_next_period_us) / 2);
                __HAL_TIM_SET_COUNTER(&htim1, 0);

                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
                __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
              }
              break;
        }
    }

    //表示用にパルスカウントと速度を計算
    rw_pulse_count_h = (_pulse_count >> 16) & 0xFFFF;
    rw_pulse_count_l = _pulse_count & 0xFFFF;
    ro_next_period_hz = (int16_t)period_to_frequency(_next_period_us);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
}

volatile int timer_1s = 0; // タイマーカウンタ
volatile int timer_10ms = 0; // タイマーカウンタ
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
      // TIM1の更新イベントは1パルス周期ごとに発生する。
      if ((rw_mode & 0x7F) == 0)
      {
        // 位置制御: 1パルスごとに現在位置を更新し、目標位置で停止する。
        int32_t move_distance = abs32(_pulse_target - _pulse_start);  // 移動距離を計算
        _pulse_count += _pulse_direction; // パルスカウントを更新
        int32_t move_progress = abs32(_pulse_count - _pulse_start); // 移動進捗を計算
        if (_pulse_count == _pulse_target)
        {
          // 目標位置に到達したらPWMとTIM1更新割り込みを停止する。
          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
          __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
          _next_period_us = 0;
          ro_status &= ~(STATUS_ACTIVE | STATUS_MOTION_MASK);
          ro_status |= STATUS_IN_POSITION;
          ro_next_period_hz = 0;
          return;
        }

        if (move_progress < ro_pulse_acc)
        {
          // 加速区間: 周期を短くしてパルス周波数を上げる。
          ro_status = (ro_status & ~STATUS_MOTION_MASK) | (1U << 4);
          _position_speed_hz += rw_ACC_ratio;
          if (_position_speed_hz > abs16(rw_FH_speed_hz))
          {
            _position_speed_hz = abs16(rw_FH_speed_hz);
          }
          _next_period_us = _pulse_direction
                          * frequency_to_period(_position_speed_hz);
        }
        else if (move_progress >= move_distance - ro_pulse_dec)
        {
          // 減速区間: 周期を長くしてパルス周波数を下げる。
          ro_status = (ro_status & ~STATUS_MOTION_MASK) | (2U << 4);
          _position_speed_hz -= rw_ACC_ratio;
          if (_position_speed_hz < rw_FL_speed_hz)
          {
            _position_speed_hz = rw_FL_speed_hz;
          }
          _next_period_us = _pulse_direction
                          * frequency_to_period(_position_speed_hz);
        }
        else
        {
          // 等速区間: 周期を一定に保つ。
          ro_status = (ro_status & ~STATUS_MOTION_MASK) | (3U << 4);
        }
        
        // 次周期を設定してTIM1の自動リロードと比較値を更新する。
        __HAL_TIM_SET_AUTORELOAD(&htim1, abs32(_next_period_us) - 1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,abs32(_next_period_us) / 2);
        ro_next_period_hz = (int16_t)period_to_frequency(_next_period_us);
        return;
      }

      int32_t target_direction = sign32(_FH_speed); // 目標速度の方向を取得
      int32_t current_direction = sign32(_next_period_us); // 現在の方向を取得
      int32_t current_period_us = abs32(_next_period_us);   // 現在の周期を取得

      // 速度制御: FH速度の符号を方向、周期の絶対値を速度として扱う。
      if (target_direction == 0 || _FL_speed <= 0 || rw_ACC_ratio <= 0)
      {
        // 目標速度がゼロの時、停止に向かうように減速処理を行う。
        if (target_direction == 0 && current_period_us < _FL_speed
            && _FL_speed > 0 && rw_ACC_ratio > 0)
        {
          //current_period_us < _FL_speedの時は減速処理を行う。
          current_period_us += rw_ACC_ratio;
          if (current_period_us > _FL_speed)
          {
            current_period_us = _FL_speed;
          }
          // 減速処理後の周期を設定する。
          _next_period_us = current_direction * current_period_us;
          __HAL_TIM_SET_AUTORELOAD(&htim1, current_period_us - 1);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                                current_period_us / 2);
          ro_next_period_hz = (int16_t)period_to_frequency(_next_period_us);
          return;
        }
        //current_period_us > _FL_speedの時はPWMを停止する。
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
        _next_period_us = 0;
        ro_status &= ~(STATUS_ACTIVE | STATUS_MOTION_MASK);
        ro_next_period_hz = 0;
        return;
      }

      // 目標速度がゼロでない場合の処理。
      // 現在速度がゼロの場合は初速周期で目標方向へ加速を開始する。
      if (current_direction == 0)
      {
        current_direction = target_direction;
        current_period_us = _FL_speed;
      }

      // 目標方向と現在方向が異なる場合は反転処理を行う。
      if (current_direction != target_direction)
      {
        ro_status = (ro_status & ~STATUS_MOTION_MASK) | (2U << 4);
        // 反転時は現在方向のまま初速周期まで減速してからDIRを切り替える。
        if (current_period_us < _FL_speed)
        {
          current_period_us += rw_ACC_ratio;
          if (current_period_us > _FL_speed)
          {
            current_period_us = _FL_speed;
          }
          _next_period_us = current_direction * current_period_us;
        }
        else
        {
          // 初速周期に達した時だけ目標方向へ切り替える。
          current_direction = target_direction;
          _next_period_us = current_direction * _FL_speed;
          current_period_us = _FL_speed;
        }
      }
      else if (current_period_us > abs32(_FH_speed))
      {
        // 加速: 周期を短くして目標速度へ近づける。
        ro_status = (ro_status & ~STATUS_MOTION_MASK) | (1U << 4);
        current_period_us -= rw_ACC_ratio;
        if (current_period_us < abs32(_FH_speed))
        {
          current_period_us = abs32(_FH_speed);
        }
        _next_period_us = current_direction * current_period_us;
      }
      else if (current_period_us < abs32(_FH_speed))
      {
        // 減速: 周期を長くして目標速度へ近づける。
        ro_status = (ro_status & ~STATUS_MOTION_MASK) | (2U << 4);
        current_period_us += rw_ACC_ratio;
        if (current_period_us > abs32(_FH_speed))
        {
          current_period_us = abs32(_FH_speed);
        }
        _next_period_us = current_direction * current_period_us;
      }
      else
      {
        // 目標速度に到達している場合の処理。
        ro_status = (ro_status & ~STATUS_MOTION_MASK) | (3U << 4);
        _next_period_us = current_direction * current_period_us;
      }

      // 現在のパルス数を更新し、DIRピンとタイマの設定を反映する。
      _pulse_count += _pulse_direction;
      HAL_GPIO_WritePin(
        DIR_PORT,
        DIR_PIN,
        (current_direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      __HAL_TIM_SET_AUTORELOAD(&htim1, current_period_us - 1);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_period_us / 2);
    }else if (htim->Instance == TIM14){
        timer_1s++;
        if (timer_1s >= 1000)
        {
            timer_1s = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        }
    }
    // ...
}
/* USER CODE END 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART2)
   {
	   _rx_buf[_rx_len] = _recv_data;
	   _rx_len++;
       HAL_UART_Receive_IT(&huart2, &_recv_data, 1);
   }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // Toggle LED to indicate error
    HAL_Delay(100); // 0.1秒待機
  }
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
