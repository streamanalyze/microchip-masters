import time
import sys
import select
import threading
import signal

import BME280 as BME280
import BMI270 as BMI270

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
#BME280_reset()
#BME280_init()
#init_weather()
#BME280_ReadCalibration()
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

BMI270.BMI270_setup()
BME280.BME280_setup()


while running:
    try:
        t_start = time.time()
        if len(signals) == 0:
            time.sleep(1)
        if "a" in signals:
            accel = BMI270.BMI270_GetAccel()
            print("%s,%f,%s,%s,%s" % ("a",time.time(), accel[0],accel[1],accel[2]))
        if "g" in signals:
            gyro = BMI270.BMI270_GetGyro()
            print("%s,%f,%s,%s,%s" % ("g",time.time(), gyro[0],gyro[1],gyro[2]))
        if "t" in signals:
            temp= BME280.BME280_GetTemperature()
            print("%s,%f,%s" % ("t",time.time(), temp))
        if "p" in signals:
            pressure= BME280.BME280_GetPressure()
            print("%s,%f,%s" % ("p",time.time(), pressure))
        if "h" in signals:
            humidity= BME280.BME280_GetHumidity()
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
