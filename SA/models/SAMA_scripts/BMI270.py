import numpy as np
import smbus
import time
from config_file import *
i2c = smbus.SMBus(2)

device_address = 0x68
BMI270_ID      = 0x00

# I2C
I2C_BUS         = 2
BMI270_I2C_PRIM_ADDR   = 0x68
BMI270_I2C_SEC_ADDR    = 0x69
# General
CHIP_ID_ADDRESS = 0x00
SENSORTIME_0    = 0x18
SENSORTIME_1    = 0x19
SENSORTIME_2    = 0x1A
INTERNAL_STATUS = 0x21
DATA_REG        = 0x0C
FIFO_CONFIG_0   = 0x48
FIFO_CONFIG_1   = 0x49
INIT_CTRL       = 0x59
INIT_ADDR_0     = 0x5B
INIT_ADDR_1     = 0x5C
INIT_DATA       = 0x5E
CMD             = 0x7E
PWR_CONF        = 0x7C
PWR_CTRL        = 0x7D

# Accelerometer
ACC_CONF        = 0x40
ACC_RANGE       = 0x41
ACC_X_7_0       = 0x0C
ACC_X_15_8      = 0x0D
ACC_Y_7_0       = 0x0E
ACC_Y_15_8      = 0x0F
ACC_Z_7_0       = 0x10
ACC_Z_15_8      = 0x11

# Gyroscope
GYR_CONF        = 0x42
GYR_RANGE       = 0x43
GYR_X_7_0       = 0x12
GYR_X_15_8      = 0x13
GYR_Y_7_0       = 0x14
GYR_Y_15_8      = 0x15
GYR_Z_7_0       = 0x16
GYR_Z_15_8      = 0x17

# Temperature
TEMP_7_0        = 0x22
TEMP_15_8       = 0x23

# Accelerometer
ACC_RANGE_2G    = 0x00      # +/- 2g
ACC_RANGE_4G    = 0x01      # +/- 4g
ACC_RANGE_8G    = 0x02      # +/- 8g
ACC_RANGE_16G   = 0x03      # +/- 16g
ACC_ODR_1600    = 0x0C      # 1600Hz
ACC_ODR_800     = 0x0B      # 800Hz
ACC_ODR_400     = 0x0A      # 400Hz
ACC_ODR_200     = 0x09      # 200Hz
ACC_ODR_100     = 0x08      # 100Hz
ACC_ODR_50      = 0x07      # 50Hz
ACC_ODR_25      = 0x06      # 25Hz
ACC_BWP_OSR4    = 0x00      # OSR4
ACC_BWP_OSR2    = 0x01      # OSR2
ACC_BWP_NORMAL  = 0x02      # Normal
ACC_BWP_CIC     = 0x03      # CIC
ACC_BWP_RES16   = 0x04      # Reserved
ACC_BWP_RES32   = 0x05      # Reserved
ACC_BWP_RES64   = 0x06      # Reserved
ACC_BWP_RES128  = 0x07      # Reserved

# Gyroscope
GYR_RANGE_2000  = 0x00      # +/- 2000dps,  16.4 LSB/dps
GYR_RANGE_1000  = 0x01      # +/- 1000dps,  32.8 LSB/dps
GYR_RANGE_500   = 0x02      # +/- 500dps,   65.6 LSB/dps
GYR_RANGE_250   = 0x03      # +/- 250dps,  131.2 LSB/dps
GYR_RANGE_125   = 0x04      # +/- 125dps,  262.4 LSB/dps
GYR_ODR_3200    = 0x0D      # 3200Hz
GYR_ODR_1600    = 0x0C      # 1600Hz
GYR_ODR_800     = 0x0B      # 800Hz
GYR_ODR_400     = 0x0A      # 400Hz
GYR_ODR_200     = 0x09      # 200Hz
GYR_ODR_100     = 0x08      # 100Hz
GYR_ODR_50      = 0x07      # 50Hz
GYR_ODR_25      = 0x06      # 25Hz
GYR_BWP_OSR4    = 0x00      # OSR4
GYR_BWP_OSR2    = 0x01      # OSR2
GYR_BWP_NORMAL  = 0x02      # Normal

BIT_0           = 0b00000001
BIT_1           = 0b00000010
BIT_2           = 0b00000100
BIT_3           = 0b00001000
BIT_4           = 0b00010000
BIT_5           = 0b00100000
BIT_6           = 0b01000000
BIT_7           = 0b10000000
LSB_MASK_8BIT   = 0x0F      # 00001111
MSB_MASK_8BIT   = 0xF0      # 11110000
FULL_MASK_8BIT  = 0xFF      # 11111111
LSB_MASK_8BIT_5 = 0x1F      # 00011111
LSB_MASK_8BIT_8 = 0x8F      # 10001111


def init_accel():

   id = BMI270_get( BMI270_ID )
   print ("ID:  ", id)
   if id != 0x24:
       print ("ERROR...")
   else:
       print ("INIT...")

def write_data( address_reg , data ):
    global device_address
    i2c.write_byte_data(device_address, address_reg, data)

def read_data(address_reg):
    global device_address
    tmp = i2c.read_byte_data(device_address,address_reg)
    return tmp


def BMI270_get(reg):
    return  read_data(reg)


def read_LSB_MSB(msb_reg,lsb_reg):
    msb = read_data(msb_reg)
    lsb = read_data(lsb_reg)
    out = ( msb << 8 ) + lsb
    return out

def BMI270_GetAccel():
  acc_value_x_lsb = read_data(ACC_X_7_0)
  acc_value_x_msb = read_data(ACC_X_15_8)
  acc_value_x = (acc_value_x_msb << 8) | acc_value_x_lsb
  
  acc_value_y_lsb = read_data(ACC_Y_7_0)
  acc_value_y_msb = read_data(ACC_Y_15_8)
  acc_value_y = (acc_value_y_msb << 8) | acc_value_y_lsb

  acc_value_z_lsb = read_data(ACC_Z_7_0)
  acc_value_z_msb = read_data(ACC_Z_15_8)
  acc_value_z = (acc_value_z_msb << 8) | acc_value_z_lsb
  
  raw_acc_data = np.array([acc_value_x, acc_value_y, acc_value_z]).astype(np.int16)
  acceleration = raw_acc_data / 32768 * 2  
  return acceleration
 
def BMI270_GetGyro():
  gyr_value_x_lsb = read_data(GYR_X_7_0)
  gyr_value_x_msb = read_data(GYR_X_15_8)
  gyr_value_x = (gyr_value_x_msb << 8) | gyr_value_x_lsb  # - GYR_CAS.factor_zx * (gyr_value_z_msb << 8 | gyr_value_z_lsb) / 2**9

  gyr_value_y_lsb = read_data(GYR_Y_7_0)
  gyr_value_y_msb = read_data(GYR_Y_15_8)
  gyr_value_y = (gyr_value_y_msb << 8) | gyr_value_y_lsb

  gyr_value_z_lsb = read_data(GYR_Z_7_0)
  gyr_value_z_msb = read_data(GYR_Z_15_8)
  gyr_value_z = (gyr_value_z_msb << 8) | gyr_value_z_lsb

  raw_gyr_data= np.array([gyr_value_x, gyr_value_y, gyr_value_z]).astype(np.int16)
  angular_velocity = np.deg2rad(1) * raw_gyr_data / 32768 * 1000
  return angular_velocity

  

def BMI270_init():
  write_data(PWR_CTRL, 0x0E)
  write_data(ACC_CONF, 0xA8)
  write_data(GYR_CONF, 0xE9)
  write_data(PWR_CONF, 0x02)
  #accel
  write_data(ACC_RANGE, ACC_RANGE_2G)
  write_data(ACC_CONF, ((read_data(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_1600))
  write_data(ACC_CONF, ((read_data(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_OSR4 << 4)))
  write_data(PWR_CTRL, (read_data(PWR_CTRL) | BIT_2))
            
  #gyro
  write_data(GYR_RANGE, GYR_RANGE_1000)
  write_data(GYR_CONF, ((read_data(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_200))
  write_data(GYR_CONF, ((read_data(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_OSR4 << 4)))
  #BMI270_1.disable_fifo_header()
  #BMI270_1.enable_data_streaming()
  #BMI270_1.enable_acc_filter_perf()
  #BMI270_1.enable_gyr_noise_perf()
  #BMI270_1.enable_gyr_filter_perf()           

def load_config_file() -> None:
  if (read_data(INTERNAL_STATUS) == 0x01):
      print(hex(0x68), " --> Initialization already done")
  else:
      print(hex(0x68), " --> Initializing...")
      write_data(PWR_CONF, 0x00)
      time.sleep(0.00045)
      write_data(INIT_CTRL, 0x00)
      for i in range(256):
          write_data(INIT_ADDR_0, 0x00)
          write_data(INIT_ADDR_1, i)
          i2c.write_i2c_block_data(0x68, INIT_DATA, bmi270_config_file[i*32:(i+1)*32])
          time.sleep(0.000020)
      write_data(INIT_CTRL, 0x01)
      time.sleep(0.02)
  print(hex(0x68), " --> Initialization status: " + '{:08b}'.format(read_data(INTERNAL_STATUS)) + "\t(00000001 --> OK)")


def BMI270_setup():
  load_config_file()
  BMI270_init()
  init_accel()


#while True:
  #out = BME280_IsMeasuring()
  #while out:
  #   out = BME280_IsMeasuring()

  #BME280_read_Measurements()
  #BMI270_GetAccel()
  #BMI270_GetGyro()
  #BME280_GetHumidity()
  #BME280_GetPressure()
  #print ("        ")
  #time.sleep(0.1)
