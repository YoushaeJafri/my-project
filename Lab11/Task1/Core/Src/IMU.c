#include "IMU.h"
struct gyro{
  uint8_t OUT_X_L;
  uint8_t OUT_X_H;
  uint8_t OUT_Y_L;
  uint8_t OUT_Y_H;
  uint8_t OUT_Z_L;
  uint8_t OUT_Z_H;
  uint8_t OUT_TEMP;

  float x_dps;
  float y_dps;
  float z_dps;
  int temperature;
  float xoffg, yoffg, zoffg;
};

struct accel{
  float accel_x;
  float accel_y;
  float accel_z;
  float xoff;
  float yoff;
  float zoff;
};

struct gyro gyro_data;
struct accel accel_data;

#define LSM_ADDR (0x19 << 1)
#define CTRL_REG1 0x20
#define OUT_TEMP_REG 0x26
# define CTRL_REG1_VAL 0b10001111

void Init_LSM(){
  HAL_I2C_Mem_Write(&hi2c1, LSM_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x67}, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Write(&hi2c1, LSM_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, HAL_MAX_DELAY);
}

void Read_LSM() {
  uint8_t low, high;
  int16_t raw_x = 0, raw_y = 0, raw_z = 0;
  
  HAL_I2C_Mem_Read(&hi2c1, LSM_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, &low, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, LSM_ADDR, 0x29 | 0x80, I2C_MEMADD_SIZE_8BIT, &high, 1, HAL_MAX_DELAY);
  raw_x = (int16_t)((high << 8) | low);
  
  HAL_I2C_Mem_Read(&hi2c1, LSM_ADDR, 0x2A | 0x80, I2C_MEMADD_SIZE_8BIT, &low, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, LSM_ADDR, 0x2B | 0x80, I2C_MEMADD_SIZE_8BIT, &high, 1, HAL_MAX_DELAY);
  raw_y = (int16_t)((high << 8) | low);
  
  HAL_I2C_Mem_Read(&hi2c1, LSM_ADDR, 0x2C | 0x80, I2C_MEMADD_SIZE_8BIT, &low, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, LSM_ADDR, 0x2D | 0x80, I2C_MEMADD_SIZE_8BIT, &high, 1, HAL_MAX_DELAY);
  raw_z = (int16_t)((high << 8) | low);
  
  accel_data.accel_x = (float)(raw_x) * 3.9f / 1000;
  accel_data.accel_y = (float)(raw_y) * 3.9f / 1000;
  accel_data.accel_z = (float)(raw_z) * 3.9f / 1000;

  accel_data.accel_x -= accel_data.xoff;
  accel_data.accel_y -= accel_data.yoff;
  accel_data.accel_z -= accel_data.zoff;
}


void gyro_init ()
{
  uint8_t tx[2];

  tx[0] = CTRL_REG1 & 0x7F;
  tx[1] = CTRL_REG1_VAL;

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

uint8_t read(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;
    tx[1] = 0x00;         

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);    
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    return rx[1];
}

void readGyro() {
  gyro_data.OUT_X_L = read(0x28);
  gyro_data.OUT_X_H = read(0x29);
  gyro_data.OUT_Y_L = read(0x2A);
  gyro_data.OUT_Y_H = read(0x2B);
  gyro_data.OUT_Z_L = read(0x2C);
  gyro_data.OUT_Z_H = read(0x2D);

  int16_t x = (int16_t)((gyro_data.OUT_X_H << 8) | gyro_data.OUT_X_L);
  int16_t y = (int16_t)((gyro_data.OUT_Y_H << 8) | gyro_data.OUT_Y_L);
  int16_t z = (int16_t)((gyro_data.OUT_Z_H << 8) | gyro_data.OUT_Z_L);

  gyro_data.x_dps = x * 0.00875f;
  gyro_data.y_dps = y * 0.00875f;
  gyro_data.z_dps = z * 0.00875f;
  
  gyro_data.x_dps -= gyro_data.xoffg;
  gyro_data.y_dps -= gyro_data.yoffg;
  gyro_data.z_dps -= gyro_data.zoffg;
}

void Offset_LSM(){
  float xofft = 0, yofft = 0, zofft = 0;
  float xofftg = 0, yofftg = 0, zofftg = 0;

  for (int i = 0; i < 20; i++) {
    Read_LSM();
    readGyro();
    xofft += accel_data.accel_x;
    yofft += accel_data.accel_y;
    zofft += accel_data.accel_z;
    xofftg += gyro_data.x_dps;
    yofftg += gyro_data.y_dps;
    zofftg += gyro_data.z_dps;
    HAL_Delay(10);
  }
  accel_data.xoff = xofft / 20;
  accel_data.yoff = yofft / 20;
  accel_data.zoff = zofft / 20;
  gyro_data.xoffg = xofftg / 20;
  gyro_data.yoffg = yofftg / 20;
  gyro_data.zoffg = zofftg / 20;
}

struct filter {
  float angle_x;
  float angle_y;
  float angle_z;
  float dt;
};

struct filter filtered_angle = {0, 0, 0, 0.1};  // dt = 100ms

void complementary_filter() {
  float accel_angle_x = atan2(accel_data.accel_y, accel_data.accel_z) * 57.2958f;
  filtered_angle.angle_x =
    0.98f * (filtered_angle.angle_x + gyro_data.x_dps * filtered_angle.dt)
  + 0.02f * accel_angle_x;
}

void Print_LSM() {
  char output[128];
  sprintf(output, "%f,%f,%f\r\n", accel_data.accel_x, gyro_data.x_dps, filtered_angle.angle_x);
  HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), HAL_MAX_DELAY);
}