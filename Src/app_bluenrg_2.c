/**
  ******************************************************************************
  * File Name          : app_bluenrg_2.c
  * Description        : Implementation file
  *
  ******************************************************************************
  *
  * COPYRIGHT 2023 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_2.h"

#include <stdlib.h>

#include "sensor.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_events.h"
#include "hci_tl.h"
#include "gatt_db.h"
#include "bluenrg_utils.h"
#include "stm32l4xx_nucleo.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>


#include "hts221_reg.h"
#include "lps22hh_reg.h"
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/**
 * 1 to send environmental and motion data when pushing the user button
 * 0 to send environmental and motion data automatically (period = 1 sec)
 */
#define USE_BUTTON (0)

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;
extern AxesRaw_t q_axes;

extern __IO uint16_t connection_handle;
extern volatile uint8_t set_connectable;
extern volatile int     connected;
uint8_t bdaddr[BDADDR_SIZE];
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;
static int16_t hts221_data_raw_humidity;
static int16_t hts221_data_raw_temperature;

static float_t hts221_humidity_perc;
static float_t hts221_temperature_degC;

static uint8_t hts221_whoamI;
static uint8_t hts221_tx_buffer[1000];

static uint32_t lps22hh_data_raw_pressure;
static int16_t lps22hh_data_raw_temperature;
static float_t lps22hh_pressure_hPa;
static float_t lps22hh_temperature_degC;
static uint8_t lps22hh_whoamI, lps22hh_rst;
static uint8_t lps22hh_tx_buffer[1000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Process(void);
static void User_Init(void);
static uint8_t Sensor_DeviceInit(void);
static void Set_Random_Environmental_Values(float *data_t, float *data_p);
static void Set_Random_Motion_Values(uint32_t cnt);
static void Reset_Motion_Values(void);

/* USER CODE BEGIN PFP */

static stmdev_ctx_t hts221_ctx;
static stmdev_ctx_t lps22hh_ctx;

static int32_t hts221_platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t hts221_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
typedef struct {
  float_t x0;
  float_t y0;
  float_t x1;
  float_t y1;
} lin_t;

float_t linear_interpolation(lin_t *lin, int16_t x)
{
  return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
                                     (lin->x0 * lin->y1)))
         / (lin->x1 - lin->x0);
}

void i2c_detect();

static int32_t lps22hh_platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t lps22hh_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

void Sensor_Init(void);
float Read_Temperature_HTS221(void);
float Read_Temperature_LPS22HH(void);
float Read_Pressure_LPS22HH(void);
float Read_Humidity_HTS221(void);


/* USER CODE END PFP */

#if PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
/**
 * @brief  This function is a utility to print the log time
 *         in the format HH:MM:SS:MSS (ST BlueNRG GUI time format)
 * @param  None
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = HAL_GetTick();
  PRINT_CSV("%02d:%02d:%02d.%03d", ms/(60*60*1000)%24, ms/(60*1000)%60, (ms/1000)%60, ms%1000);
}
#endif

void MX_BlueNRG_2_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BlueNRG_2_Init_PreTreatment */

  /* USER CODE END BlueNRG_2_Init_PreTreatment */

  /* Initialize the peripherals and the BLE Stack */
  uint8_t ret;

  User_Init();
  Sensor_Init();

  /* Get the User Button initial state */
  user_button_init_state = BSP_PB_GetState(BUTTON_KEY);

  hci_init(APP_UserEvtRx, NULL);

  PRINT_DBG("BlueNRG-2 SensorDemo_BLESensor-App Application\r\n");

  /* Init Sensor Device */
  ret = Sensor_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS)
  {
    BSP_LED_On(LED2);
    PRINT_DBG("SensorDeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }

  PRINT_DBG("BLE Stack Initialized & Device Configured\r\n");

  /* USER CODE BEGIN BlueNRG_2_Init_PostTreatment */

  /* USER CODE END BlueNRG_2_Init_PostTreatment */
}

/*
 * BlueNRG-2 background task
 */
void MX_BlueNRG_2_Process(void)
{
  /* USER CODE BEGIN BlueNRG_2_Process_PreTreatment */

  /* USER CODE END BlueNRG_2_Process_PreTreatment */

  hci_user_evt_proc();
  User_Process();

  /* USER CODE BEGIN BlueNRG_2_Process_PostTreatment */

  /* USER CODE END BlueNRG_2_Process_PostTreatment */
}

/**
 * @brief  Initialize User process
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  BSP_LED_Init(LED2);

  BSP_COM_Init(COM1);
}

/**
 * @brief  Initialize the device sensors
 *
 * @param  None
 * @retval None
 */
uint8_t Sensor_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {SENSOR_DEMO_NAME};
  uint8_t  hwVersion;
  uint16_t fwVersion;
  uint8_t  bdaddr_len_out;
  uint8_t  config_data_stored_static_random_address = 0x80; /* Offset of the static random address stored in NVM */

  /* Sw reset of the device */
  hci_reset();
  /**
   *  To support both the BlueNRG-2 and the BlueNRG-2N a minimum delay of 2000ms is required at device boot
   */
  HAL_Delay(2000);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  PRINT_DBG("HWver %d\nFWver %d\r\n", hwVersion, fwVersion);

  ret = aci_hal_read_config_data(config_data_stored_static_random_address,
                                 &bdaddr_len_out, bdaddr);

  if (ret) {
    PRINT_DBG("Read Static Random address failed.\r\n");
  }

  if ((bdaddr[5] & 0xC0) != 0xC0) {
    PRINT_DBG("Static Random address not well formed.\r\n");
    while(1);
  }

  /* Set the TX power -2 dBm */
  aci_hal_set_tx_power_level(1, 4);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("Error in aci_hal_set_tx_power_level() 0x%04x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_hal_set_tx_power_level() --> SUCCESS\r\n");
  }

  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gatt_init() --> SUCCESS\r\n");
  }

  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle,
                     &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gap_init() --> SUCCESS\r\n");
  }

  /* Update device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name),
                                   device_name);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_update_char_value() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gatt_update_char_value() --> SUCCESS\r\n");
  }

  /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x (new API prototype) */
  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7,
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456,
                                               0x00);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_set_authentication_requirement()failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gap_set_authentication_requirement() --> SUCCESS\r\n");
  }

  PRINT_DBG("BLE Stack Initialized with SUCCESS\r\n");

  ret = Add_HWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("BlueNRG2 HW service added successfully.\r\n");
  }
  else
  {
    PRINT_DBG("Error while adding BlueNRG2 HW service: 0x%02x\r\n", ret);
    while(1);
  }

  ret = Add_SWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     PRINT_DBG("BlueNRG2 SW service added successfully.\r\n");
  }
  else
  {
     PRINT_DBG("Error while adding BlueNRG2 HW service: 0x%02x\r\n", ret);
     while(1);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  User Process
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
{

  static uint32_t counter = 0;

  /* Make the device discoverable */
  if(set_connectable)
  {
    Set_DeviceConnectable();
    set_connectable = FALSE;
  }

  /*  Update sensor value */
#if USE_BUTTON
  /* Check if the user has pushed the button */
  if (user_button_pressed)
  {
    /* Debouncing */
    HAL_Delay(50);

    /* Wait until the User Button is released */
    while (BSP_PB_GetState(BUTTON_KEY) == !user_button_init_state);

    /* Debouncing */
    HAL_Delay(50);
#endif
    BSP_LED_Toggle(LED2);

    if (connected)
    {
      /* Set a random seed */
      srand(HAL_GetTick());

      /* Update emulated Environmental data */
      float temp = Read_Temperature_LPS22HH();
      float press = Read_Pressure_LPS22HH();
      float hum = Read_Humidity_HTS221();
      printf("temperature: %.2f\r\npressure: %.2f\r\nhumidity: %.2f\r\n", temp, press, hum);


      Environmental_Update((int32_t)(press * 100), (int16_t)(temp * 10), (int16_t)(hum * 10));



      /* Update emulated Acceleration, Gyroscope and Sensor Fusion data */
      Set_Random_Motion_Values(counter);
      Acc_Update(&x_axes, &g_axes, &m_axes);
      Quat_Update(&q_axes);

      counter ++;
      if (counter == 40) {
        counter = 0;
        Reset_Motion_Values();
      }
#if !USE_BUTTON
      HAL_Delay(1000); /* wait 1 sec before sending new data */
#endif
    }
#if USE_BUTTON
    /* Reset the User Button flag */
    user_button_pressed = 0;
  }
#endif
}

/**
 * @brief  Set random values for all environmental sensor data
 *
 * @param  float pointer to temperature data
 * @param  float pointer to pressure data
 * @retval None
 */
static void Set_Random_Environmental_Values(float *data_t, float *data_p)
{
  *data_t = 20.0 + ((uint64_t)rand()*5)/RAND_MAX;     /* T sensor emulation */
  *data_p = 1000.0 + ((uint64_t)rand()*80)/RAND_MAX; /* P sensor emulation */
}

/**
 * @brief  Set random values for all motion sensor data
 *
 * @param  uint32_t counter for changing the rotation direction
 * @retval None
 */
static void Set_Random_Motion_Values(uint32_t cnt)
{
  /* Update Acceleration, Gyroscope and Sensor Fusion data */
  if (cnt < 20) {
    x_axes.AXIS_X +=  (10  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
    x_axes.AXIS_Y += -(10  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
    x_axes.AXIS_Z +=  (10  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
    g_axes.AXIS_X +=  (100 + ((uint64_t)rand()*2*cnt)/RAND_MAX);
    g_axes.AXIS_Y += -(100 + ((uint64_t)rand()*4*cnt)/RAND_MAX);
    g_axes.AXIS_Z +=  (100 + ((uint64_t)rand()*6*cnt)/RAND_MAX);
    m_axes.AXIS_X +=  (3  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
    m_axes.AXIS_Y += -(3  + ((uint64_t)rand()*4*cnt)/RAND_MAX);
    m_axes.AXIS_Z +=  (3  + ((uint64_t)rand()*5*cnt)/RAND_MAX);

    q_axes.AXIS_X -= (100  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
    q_axes.AXIS_Y += (100  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
    q_axes.AXIS_Z -= (100  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
  }
  else {
    x_axes.AXIS_X += -(10  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
    x_axes.AXIS_Y +=  (10  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
    x_axes.AXIS_Z += -(10  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
    g_axes.AXIS_X += -(100 + ((uint64_t)rand()*2*cnt)/RAND_MAX);
    g_axes.AXIS_Y +=  (100 + ((uint64_t)rand()*4*cnt)/RAND_MAX);
    g_axes.AXIS_Z += -(100 + ((uint64_t)rand()*6*cnt)/RAND_MAX);
    m_axes.AXIS_X += -(3  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
    m_axes.AXIS_Y +=  (3  + ((uint64_t)rand()*9*cnt)/RAND_MAX);
    m_axes.AXIS_Z += -(3  + ((uint64_t)rand()*3*cnt)/RAND_MAX);

    q_axes.AXIS_X += (200 + ((uint64_t)rand()*7*cnt)/RAND_MAX);
    q_axes.AXIS_Y -= (150 + ((uint64_t)rand()*3*cnt)/RAND_MAX);
    q_axes.AXIS_Z += (10  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
  }
}

/**
 * @brief  Reset values for all motion sensor data
 *
 * @param  None
 * @retval None
 */
static void Reset_Motion_Values(void)
{
  x_axes.AXIS_X = (x_axes.AXIS_X)%2000 == 0 ? -x_axes.AXIS_X : 10;
  x_axes.AXIS_Y = (x_axes.AXIS_Y)%2000 == 0 ? -x_axes.AXIS_Y : -10;
  x_axes.AXIS_Z = (x_axes.AXIS_Z)%2000 == 0 ? -x_axes.AXIS_Z : 10;
  g_axes.AXIS_X = (g_axes.AXIS_X)%2000 == 0 ? -g_axes.AXIS_X : 100;
  g_axes.AXIS_Y = (g_axes.AXIS_Y)%2000 == 0 ? -g_axes.AXIS_Y : -100;
  g_axes.AXIS_Z = (g_axes.AXIS_Z)%2000 == 0 ? -g_axes.AXIS_Z : 100;
  m_axes.AXIS_X = (g_axes.AXIS_X)%2000 == 0 ? -m_axes.AXIS_X : 3;
  m_axes.AXIS_Y = (g_axes.AXIS_Y)%2000 == 0 ? -m_axes.AXIS_Y : -3;
  m_axes.AXIS_Z = (g_axes.AXIS_Z)%2000 == 0 ? -m_axes.AXIS_Z : 3;
  q_axes.AXIS_X = -q_axes.AXIS_X;
  q_axes.AXIS_Y = -q_axes.AXIS_Y;
  q_axes.AXIS_Z = -q_axes.AXIS_Z;
}

/**
 * @brief  Get hardware and firmware version
 *
 * @param  Hardware version
 * @param  Firmware version
 * @retval Status
 */
uint8_t getBlueNRGVersion(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;

  status = hci_read_local_version_information(&hci_version, &hci_revision, &lmp_pal_version,
				                              &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
    *fwVersion = (hci_revision & 0xFF) << 8;              // Major Version Number
    *fwVersion |= ((lmp_pal_subversion >> 4) & 0xF) << 4; // Minor Version Number
    *fwVersion |= lmp_pal_subversion & 0xF;               // Patch Version Number
  }
  return status;
}

/**
 * @brief  BSP Push Button callback
 *
 * @param  Button Specifies the pin connected EXTI line
 * @retval None
 */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Set the User Button flag */
  user_button_pressed = 1;
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/
/**
 * @brief  This event indicates that a new connection has been created
 *
 * @param  See file bluenrg1_events.h
 * @retval See file bluenrg1_events.h
 */
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  connected = TRUE;
  connection_handle = Connection_Handle;

  BSP_LED_Off(LED2); //activity led
}

/**
 * @brief  This event occurs when a connection is terminated
 *
 * @param  See file bluenrg1_events.h
 * @retval See file bluenrg1_events.h
 */
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connected = FALSE;
  /* Make the device connectable again */
  set_connectable = TRUE;
  connection_handle =0;
  PRINT_DBG("Disconnected\r\n");

  BSP_LED_On(LED2); //activity led
}

/**
 * @brief  This event is given when a read request is received
 *         by the server from the client
 * @param  See file bluenrg1_events.h
 * @retval See file bluenrg1_events.h
 */
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  Read_Request_CB(Attribute_Handle);
}

void Sensor_Init(void) {

    hts221_ctx.write_reg = hts221_platform_write;
    hts221_ctx.read_reg = hts221_platform_read;
    hts221_ctx.mdelay = HAL_Delay;
    hts221_ctx.handle = &hi2c1;


    lps22hh_ctx.write_reg = lps22hh_platform_write;
    lps22hh_ctx.read_reg = lps22hh_platform_read;
    lps22hh_ctx.mdelay = HAL_Delay;
    lps22hh_ctx.handle = &hi2c1;


    hts221_power_on_set(&hts221_ctx, PROPERTY_ENABLE);
    hts221_block_data_update_set(&hts221_ctx, PROPERTY_ENABLE);
    hts221_data_rate_set(&hts221_ctx, HTS221_ODR_1Hz);


    lps22hh_reset_set(&lps22hh_ctx, PROPERTY_ENABLE);
    do {
        lps22hh_reset_get(&lps22hh_ctx, &lps22hh_rst);
    } while (lps22hh_rst);

    lps22hh_block_data_update_set(&lps22hh_ctx, PROPERTY_ENABLE);
    lps22hh_data_rate_set(&lps22hh_ctx, LPS22HH_10_Hz_LOW_NOISE);
}

float Read_Humidity_HTS221(void) {
    int16_t raw;
    lin_t lin_hum;
    float hum;

    hts221_hum_adc_point_0_get(&hts221_ctx, &lin_hum.x0);
    hts221_hum_rh_point_0_get(&hts221_ctx, &lin_hum.y0);
    hts221_hum_adc_point_1_get(&hts221_ctx, &lin_hum.x1);
    hts221_hum_rh_point_1_get(&hts221_ctx, &lin_hum.y1);

    hts221_humidity_raw_get(&hts221_ctx, &raw);
    hum = linear_interpolation(&lin_hum, raw);

    if (hum < 0) {
    	hum = 0;
    }
    if (hum > 100) {
    	hum = 100;
    }

    return hum;
}


float Read_Temperature_HTS221(void) {
    int16_t raw;
    lin_t lin_temp;
    float temp;


    hts221_temp_adc_point_0_get(&hts221_ctx, &lin_temp.x0);
    hts221_temp_deg_point_0_get(&hts221_ctx, &lin_temp.y0);
    hts221_temp_adc_point_1_get(&hts221_ctx, &lin_temp.x1);
    hts221_temp_deg_point_1_get(&hts221_ctx, &lin_temp.y1);

    hts221_temperature_raw_get(&hts221_ctx, &raw);
    temp = ((lin_temp.y1 - lin_temp.y0) * raw + (lin_temp.x1 * lin_temp.y0 - lin_temp.x0 * lin_temp.y1)) / (lin_temp.x1 - lin_temp.x0);

    return temp;
}

float Read_Pressure_LPS22HH(void) {
    uint32_t raw;
    float pressure;

    lps22hh_pressure_raw_get(&lps22hh_ctx, &raw);
    pressure = lps22hh_from_lsb_to_hpa(raw);
    return pressure;
}

float Read_Temperature_LPS22HH(void) {
    int16_t raw;
    float temp;

    lps22hh_temperature_raw_get(&lps22hh_ctx, &raw);
    temp = lps22hh_from_lsb_to_celsius(raw);
    return temp;
}
static int32_t hts221_platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

  return 0;
}
static int32_t hts221_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}


static int32_t lps22hh_platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t lps22hh_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
