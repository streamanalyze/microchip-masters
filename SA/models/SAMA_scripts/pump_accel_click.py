import numpy as np
import smbus
import time
import sys
import select
import threading
import signal
from config_file import *
i2c = smbus.SMBus(2)

device_address = 0x76
BMI_ADDRESS = 0x68
BMI270_ID           = 0x00
tmp_data = []

C_TMP = [ 0, 0, 0 ]
C_HUM = [ 0, 0, 0, 0, 0, 0 ]
C_PRE = [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ]

HUMIDITY_DATA = 0
TEMPERATURE_DATA = 0
PRESSURE_DATA = 0
FINE = 0

BME280_PRESSURE     = 0xF7
BME280_ID           = 0xD0
BME280_RESET        = 0xB6
BME280_RST_REG      = 0xE0
BME280_STATUS       = 0xF3
BME280_MEAS_REG     = 0xF4
BME280_HUMIDITY_REG = 0xF2
BME280_CONFIG_REG   = 0xF5

BME280_TEMPERATURE_0_LSB_REG   = 0x88
BME280_TEMPERATURE_0_MSB_REG   = 0x89
BME280_TEMPERATURE_1_LSB_REG   = 0x8A
BME280_TEMPERATURE_1_MSB_REG   = 0x8B
BME280_TEMPERATURE_2_LSB_REG   = 0x8C
BME280_TEMPERATURE_2_MSB_REG   = 0x8D

BME280_PRESSURE_0_LSB_REG      = 0x8E
BME280_PRESSURE_0_MSB_REG      = 0x8F
BME280_PRESSURE_1_LSB_REG      = 0x90
BME280_PRESSURE_1_MSB_REG      = 0x91
BME280_PRESSURE_2_LSB_REG      = 0x92
BME280_PRESSURE_2_MSB_REG      = 0x93
BME280_PRESSURE_3_LSB_REG      = 0x94
BME280_PRESSURE_3_MSB_REG      = 0x95
BME280_PRESSURE_4_LSB_REG      = 0x96
BME280_PRESSURE_4_MSB_REG      = 0x97
BME280_PRESSURE_5_LSB_REG      = 0x98
BME280_PRESSURE_5_MSB_REG      = 0x99
BME280_PRESSURE_6_LSB_REG      = 0x9A
BME280_PRESSURE_6_MSB_REG      = 0x9B
BME280_PRESSURE_7_LSB_REG      = 0x9C
BME280_PRESSURE_7_MSB_REG      = 0x9D
BME280_PRESSURE_8_LSB_REG      = 0x9E
BME280_PRESSURE_8_MSB_REG      = 0x9F

BME280_HUMIDITY_0_CHR_REG      = 0xA1
BME280_HUMIDITY_1_LSB_REG      = 0xE1
BME280_HUMIDITY_1_MSB_REG      = 0xE2
BME280_HUMIDITY_2_CHR_REG      = 0xE3
BME280_HUMIDITY_3_MSB_REG      = 0xE4
BME280_HUMIDITY_3_LSB_REG      = 0xE5
BME280_HUMIDITY_4_MSB_REG      = 0xE6
BME280_HUMIDITY_5_CHR_REG      = 0xE7

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
def init_weather():

   id = BME280_get( BME280_ID )
   print ("ID:  ", id)
   if id != 0x60:
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

def write_data_bmi( address_reg , data ):
    device_address = BMI_ADDRESS
    i2c.write_byte_data(device_address, address_reg, data)

def read_data_bmi(address_reg):
    device_address = BMI_ADDRESS
    tmp = i2c.read_byte_data(device_address,address_reg)
    return tmp

def BME280_read_Measurements():
    global device_address
    global BME280_PRESSURE
    global tmp_data
    global HUMIDITY_DATA,TEMPERATURE_DATA,PRESSURE_DATA

    tmp_data = i2c.read_i2c_block_data(device_address,BME280_PRESSURE,8) #size 8 byte
    # Humidity
    HUMIDITY_DATA  =  tmp_data[7]
    HUMIDITY_DATA |=  tmp_data[6]  << 8
    #Temperature
    TEMPERATURE_DATA = tmp_data[5] >> 4
    TEMPERATURE_DATA |= tmp_data[4] << 4
    TEMPERATURE_DATA |= tmp_data[3] << 12
    #Pressure
    PRESSURE_DATA = tmp_data[2] >> 4
    PRESSURE_DATA |= tmp_data[1] << 4
    PRESSURE_DATA |= tmp_data[0] << 12

def BMI270_get(reg):
    return  read_data_bmi(reg)

def BME280_get(reg):
    return  read_data(reg)

def BME280_reset():
    global BME280_RST_REG
    global BME280_RESET
    write_data(BME280_RST_REG,BME280_RESET)

def read_LSB_MSB(msb_reg,lsb_reg):
    msb = read_data(msb_reg)
    lsb = read_data(lsb_reg)
    out = ( msb << 8 ) + lsb
    return out

def BME280_ReadCalibration():
   global C_TMP, C_PRE, C_HUM
   global BME280_TEMPERATURE_0_MSB_REG,BME280_TEMPERATURE_0_LSB_REG
   global BME280_TEMPERATURE_1_MSB_REG,BME280_TEMPERATURE_1_LSB_REG
   global BME280_TEMPERATURE_2_MSB_REG,BME280_TEMPERATURE_2_LSB_REG
   global BME280_PRESSURE_0_MSB_REG,BME280_PRESSURE_0_LSB_REG
   global BME280_PRESSURE_1_MSB_REG,BME280_PRESSURE_1_LSB_REG
   global BME280_PRESSURE_2_MSB_REG,BME280_PRESSURE_2_LSB_REG
   global BME280_PRESSURE_3_MSB_REG,BME280_PRESSURE_3_LSB_REG
   global BME280_PRESSURE_4_MSB_REG,BME280_PRESSURE_4_LSB_REG
   global BME280_PRESSURE_5_MSB_REG,BME280_PRESSURE_5_LSB_REG
   global BME280_PRESSURE_6_MSB_REG,BME280_PRESSURE_6_LSB_REG
   global BME280_PRESSURE_7_MSB_REG,BME280_PRESSURE_7_LSB_REG
   global BME280_PRESSURE_8_MSB_REG,BME280_PRESSURE_8_LSB_REG
   global BME280_HUMIDITY_0_CHR_REG,BME280_HUMIDITY_1_MSB_REG,BME280_HUMIDITY_1_LSB_REG
   global BME280_HUMIDITY_2_CHR_REG,BME280_HUMIDITY_3_MSB_REG,BME280_HUMIDITY_3_LSB_REG
   global BME280_HUMIDITY_4_MSB_REG,BME280_HUMIDITY_3_LSB_REG,BME280_HUMIDITY_5_CHR_REG

   C_TMP[0] = read_LSB_MSB(BME280_TEMPERATURE_0_MSB_REG, BME280_TEMPERATURE_0_LSB_REG )
   if(C_TMP[0] < 26000):
       C_TMP[0] = 28440
   C_TMP[1] = read_LSB_MSB(BME280_TEMPERATURE_1_MSB_REG, BME280_TEMPERATURE_1_LSB_REG )
   C_TMP[2] = read_LSB_MSB(BME280_TEMPERATURE_2_MSB_REG, BME280_TEMPERATURE_2_LSB_REG )
   C_PRE[0] = read_LSB_MSB(BME280_PRESSURE_0_MSB_REG, BME280_PRESSURE_0_LSB_REG)
   C_PRE[1] = -10677
   C_PRE[2] = read_LSB_MSB(BME280_PRESSURE_2_MSB_REG, BME280_PRESSURE_2_LSB_REG)
   C_PRE[3] = read_LSB_MSB(BME280_PRESSURE_3_MSB_REG, BME280_PRESSURE_3_LSB_REG)
   C_PRE[4] = -88
   C_PRE[5] = -10
   C_PRE[6] = read_LSB_MSB(BME280_PRESSURE_6_MSB_REG, BME280_PRESSURE_6_LSB_REG)
   C_PRE[7]= -10230
   C_PRE[8] = read_LSB_MSB(BME280_PRESSURE_8_MSB_REG, BME280_PRESSURE_8_LSB_REG)
   C_HUM[0] = read_data(BME280_HUMIDITY_0_CHR_REG)
   C_HUM[1] = read_LSB_MSB(BME280_HUMIDITY_1_MSB_REG, BME280_HUMIDITY_1_LSB_REG)
   C_HUM[2] = read_data(BME280_HUMIDITY_2_CHR_REG)
   C_HUM[3] = ( read_data(BME280_HUMIDITY_3_MSB_REG) << 4 ) | ( read_data(BME280_HUMIDITY_3_LSB_REG) & 0xF )
   C_HUM[4] = ( read_data(BME280_HUMIDITY_4_MSB_REG) << 4 ) | ( read_data(BME280_HUMIDITY_3_LSB_REG) >> 4 )
   C_HUM[5] = read_data(BME280_HUMIDITY_5_CHR_REG)


def BME280_Set_Pressure(value):
   global BME280_MEAS_REG
   getMeasurement = BME280_get( BME280_MEAS_REG )
   getMeasurement &= ~ 0x1C
   getMeasurement |= value << 2
   write_data( BME280_MEAS_REG, getMeasurement )

def BME280_Set_Temperature(value):
    global BME280_MEAS_REG
    getMeasurement = BME280_get( BME280_MEAS_REG )
    getMeasurement &= ~ 0xE0
    getMeasurement |= value << 5
    write_data( BME280_MEAS_REG, getMeasurement )

def BME280_Set_Humidity(value):
    global BME280_HUMIDITY_REG
    write_data( BME280_HUMIDITY_REG, value )

def BME280_Set_Mode(value):
   global BME280_MEAS_REG
   getMeasurement = BME280_get( BME280_MEAS_REG )
   getMeasurement |= value
   write_data( BME280_MEAS_REG, getMeasurement )

def BME280_IsMeasuring():
   status = BME280_get( BME280_STATUS )
   return status &  0x08

def BME280_Compensate_Temperature():
   global TEMPERATURE_DATA
   global C_TMP,FINE
   temp1 = (TEMPERATURE_DATA/8) - (C_TMP[0]*2 )
   temp1=  ( temp1 * C_TMP[1] ) / 2048
   temp2 = (TEMPERATURE_DATA /16) - C_TMP[0]
   temp2 = temp2 * temp2 / 4096
   temp2 = (temp2 * 50) /16384
   FINE = temp1 + temp2
   T = (FINE * 5 + 128) / 256
   return T

def BME280_Compensate_Humidity():
   global HUMIDITY_DATA
   global FINE,C_HUM
   h1 = FINE - 76800
   h2 = (HUMIDITY_DATA * 16384) - (C_HUM[3] * 1024 *1024) - (C_HUM[4] * h1)
   h2 = (h2 + 16384)/ 32768
   h3 = (((((h1 * C_HUM[5]) / 1024)*(((h1 * C_HUM[2]) / 2048) + 32768)) /1024 + 2097152)* C_HUM[1] + 8192)/ 16384
   h1 = h2 * h3
   h1 = (h1 - (((((h1 / 32768) * (h1 / 32768))/ 128) * C_HUM[0]) / 16))
   if(h1 < 0):
       h1 = 0
   if (h1 > 419430400):
       h1 = 419430400
   h1= h1 / 4096
   return h1

def BME280_Compensate_Pressure():
   global C_PRE,FINE,PRESSURE_DATA
   press1 = (int(FINE) >>1) - 64000
   press2 = (((press1>>2) * (press1>>2)) >> 11 ) * (C_PRE[5])
   press2 = press2 + ((press1*(C_PRE[4]))<<1)
   press2 = (press2 >> 2)+((C_PRE[3])<<16)
   press1 = (((C_PRE[2] * (((press1>>2) * (press1>>2)) >> 13 )) >> 3) + (((C_PRE[1]) * press1)>>1))>>18
   press1 =((((32768+press1))*(C_PRE[0]))>>15)
   if (press1 == 0):
        return 0
   P = int(((((1048576)-PRESSURE_DATA)-(press2>>12)))*3125)
   if (P < 0x80000000):
        P = (P << 1) / press1
   else:
        P = (P / press1) * 2

   press1 = ((C_PRE[8]) * ((((int(P)>>3) * (int(P)>>3))>>13)))>>12
   press2 = ((int(P)>>2) * (C_PRE[7])) >> 13
   P = (P + ((press1 + press2 + C_PRE[6]) >> 4))
   return P

def BME280_GetTemperature():
   print ("Temperature:  ", BME280_Compensate_Temperature() / 100)


def BME280_GetHumidity():
   print ("Humidity:  ", BME280_Compensate_Humidity()  / 1020)

def BME280_GetPressure():
   print ("Pressure:  ", BME280_Compensate_Pressure()  / 100)

def BME280_init():
    BME280_Set_Pressure(0x05)
    BME280_Set_Temperature(0x02)
    BME280_Set_Humidity(0x01)
    BME280_Set_Mode(0x03)

def BME280_uglyPressure():
  while BME280_IsMeasuring():
    time.sleep(0.001)
  BME280_read_Measurements()
  return BME280_Compensate_Pressure()/100

def BME280_uglyHumidity():
  while BME280_IsMeasuring():
    time.sleep(0.001)
  BME280_read_Measurements()
  return BME280_Compensate_Humidity()/1020

def BME280_uglyTemperature():
  while BME280_IsMeasuring():
    time.sleep(0.001)
  BME280_read_Measurements()
  return BME280_Compensate_Temperature()/100


def BMI270_GetAccel():
  acc_value_x_lsb = read_data(ACC_X_7_0)
  acc_value_x_msb = read_data_bmi(ACC_X_15_8)
  acc_value_x = (acc_value_x_msb << 8) | acc_value_x_lsb
  acc_value_y_lsb = read_data_bmi(ACC_Y_7_0)
  acc_value_y_msb = read_data_bmi(ACC_Y_15_8)
  acc_value_y = (acc_value_y_msb << 8) | acc_value_y_lsb

  acc_value_z_lsb = read_data_bmi(ACC_Z_7_0)
  acc_value_z_msb = read_data_bmi(ACC_Z_15_8)
  acc_value_z = (acc_value_z_msb << 8) | acc_value_z_lsb
  
  raw_acc_data = np.array([acc_value_x, acc_value_y, acc_value_z]).astype(np.int16)
  acceleration = raw_acc_data / 32768 * 4  
  #print(acceleration)
  return acceleration
 
def BMI270_GetGyro():
  gyr_value_x_lsb = read_data_bmi(GYR_X_7_0)
  gyr_value_x_msb = read_data_bmi(GYR_X_15_8)
  gyr_value_x = (gyr_value_x_msb << 8) | gyr_value_x_lsb  # - GYR_CAS.factor_zx * (gyr_value_z_msb << 8 | gyr_value_z_lsb) / 2**9

  gyr_value_y_lsb = read_data_bmi(GYR_Y_7_0)
  gyr_value_y_msb = read_data_bmi(GYR_Y_15_8)
  gyr_value_y = (gyr_value_y_msb << 8) | gyr_value_y_lsb

  gyr_value_z_lsb = read_data_bmi(GYR_Z_7_0)
  gyr_value_z_msb = read_data_bmi(GYR_Z_15_8)
  gyr_value_z = (gyr_value_z_msb << 8) | gyr_value_z_lsb

  raw_gyr_data= np.array([gyr_value_x, gyr_value_y, gyr_value_z]).astype(np.int16)
  
  angular_velocity = np.deg2rad(1) * raw_gyr_data / 32768 * 1000

  #print(angular_velocity)
  return angular_velocity

def BMI270_init():
  write_data_bmi(PWR_CTRL, 0x0E)
  write_data_bmi(ACC_CONF, 0xA8)
  write_data_bmi(GYR_CONF, 0xE9)
  write_data_bmi(PWR_CONF, 0x02)
  #accel
  write_data_bmi(ACC_RANGE, ACC_RANGE_4G)
  write_data_bmi(ACC_CONF, ((read_data_bmi(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_1600))
  write_data_bmi(ACC_CONF, ((read_data_bmi(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_OSR4 << 4)))
  write_data_bmi(PWR_CTRL, (read_data_bmi(PWR_CTRL) | BIT_2))
            
  #gyro
  write_data_bmi(GYR_RANGE, GYR_RANGE_1000)
  write_data_bmi(GYR_CONF, ((read_data_bmi(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_200))
  write_data_bmi(GYR_CONF, ((read_data_bmi(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_OSR4 << 4)))
  #BMI270_1.disable_fifo_header()
  #BMI270_1.enable_data_streaming()
  #BMI270_1.enable_acc_filter_perf()
  #BMI270_1.enable_gyr_noise_perf()
  #BMI270_1.enable_gyr_filter_perf()           

def load_config_file() -> None:
  if (read_data_bmi(INTERNAL_STATUS) == 0x01):
      print(hex(0x68), " --> Initialization already done")
  else:
      print(hex(0x68), " --> Initializing...")
      write_data_bmi(PWR_CONF, 0x00)
      time.sleep(0.00045)
      write_data_bmi(INIT_CTRL, 0x00)
      for i in range(256):
          write_data_bmi(INIT_ADDR_0, 0x00)
          write_data_bmi(INIT_ADDR_1, i)
          i2c.write_i2c_block_data(0x68, INIT_DATA, bmi270_config_file[i*32:(i+1)*32])
          time.sleep(0.000020)
      write_data_bmi(INIT_CTRL, 0x01)
      time.sleep(0.02)
  print(hex(0x68), " --> Initialization status: " + '{:08b}'.format(read_data_bmi(INTERNAL_STATUS)) + "\t(00000001 --> OK)")

# Void function to swallow unwanted output
def _void_f(*args,**kwargs):
    pass

# Global container that keeps track of active signals
signals = []

# Check if activate signal from start
if len(sys.argv) > 1:
    signals = [s for s in sys.argv[1:]]

running = True

# Thread listening to standard in for adding/removing signals
def thread_function(name):
    for line in sys.stdin:
        print("line: "+line, file=sys.stderr)
        if line:
            if(line[0] == "+" and line[1:len(line):1] not in signals):
                signals.append(line[1:len(line)-1:1])
            elif(line[0] == "-"):
                signals.remove(line[1:len(line)-1:1])
    running = False
    quit()

# Start listening on standard in
x = threading.Thread(target=thread_function, args=(1,))
x.start()

dt = 0.02   # How often inner loop is executed
tick = 0


# start main
#load_config_file()
#BMI270_init()
#init_accel()
BME280_reset()
BME280_init()
init_weather()
BME280_ReadCalibration()
#time.sleep(2)

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

while running:
    try:
        t_start = time.time()
        if len(signals) == 0:
            time.sleep(1)
        if "a" in signals:
            accel = BMI270_GetAccel()
            print("%s,%f,%s,%s,%s" % ("a",time.time(), accel[0],accel[1],accel[2]))
        if "g" in signals:
            gyro = BMI270_GetGyro()
            print("%s,%f,%s,%s,%s" % ("g",time.time(), gyro[0],gyro[1],gyro[2]))
        if "t" in signals:
            temp= BME280_uglyTemperature()
            print("%s,%f,%s" % ("t",time.time(), temp))
        if "p" in signals:
            pressure= BME280_uglyPressure()
            print("%s,%f,%s" % ("p",time.time(), pressure))
        if "h" in signals:
            humidity= BME280_uglyHumidity()
            print("%s,%f,%s" % ("h",time.time(), humidity))  
        t_end = time.time()
        t_left = dt - (t_end - t_start)
        tick = (tick+1) % 21
        if tick % 2 == 0:      # Flush every 2nd loop
            sys.stdout.flush()
        if t_left > 0:
            time.sleep(t_left)
    except KeyboardInterrupt:
        pass
    except Exception:
        sys.stdout.write = _void_f
        sys.stdout.flush = _void_f
        sys.stderr.write = _void_f
        sys.stderr.flush = _void_f
        quit()
        pass

quit()
