# GOPIGO3 SPI BUS ERROR TEST

### Two programs are launched:
- spitest.py  checks gopigo3.get_voltage_battery() every 0.001  seconds
- spitest2.py checks gopigo3.get_voltage_battery() every 0.0013 seconds
- use special instrumented spi_gopigo3.py that counts "No SPI RESPONSE" errors in spi_read_8(), spi_read_16(), and spi_read32()  
  get_voltage_battery() uses spi_read_16()


### Results

- Test 1 with spitest2.py in ROS2/Ubuntu22 Docker container, spitest.py in PiOS Bookworm:  
  21 SPI errors reported in 800,000 voltage checks between the two processes.  

- Test 2 with both spitest.py and spitest2.py in PiOS Bookworm  
  29 SPI errors reported in 820,000 voltage checks between the two processes.  


## MUTEX PROTECTED TRANSER_ARRAY()

### Two programs are launched:
- spi_mutex_user.py  checks gopigo3.get_voltage_battery() every 0.001  seconds
- spi_mutex_user2.py checks gopigo3.get_voltage_battery() every 0.0013 seconds
- use special instrumented spi_mutex_gopigo3.py that uses DI_Mutex("SPI") and  
  and counts "No SPI RESPONSE" errors in spi_read_8(), spi_read_16(), and spi_read32()  
  get_voltage_battery() uses spi_read_16()


### Results

- Test 1  
  - both spi_mutex_user.py and spi_mutex_user2.py in PiOS Bookworm  
  - only acquire() and release() (no delay after acquire)  
  11 SPI errors reported in 800,000 voltage checks between the two processes.  

- Test 2  
  - both spi_mutex_user.py and spi_mutex_user2.py in PiOS Bookworm  
  - 0.001s delay after acquire()  
   9 SPI errors reported in 800,000 voltage checks between the two processes.  

- Test 3  
  - both spi_mutex_user.py and spi_mutex_user2.py in PiOS Bookworm  
  - 0.0001s delay after acquire()  
  11 SPI errors reported in 800,000 voltage checks between the two processes.  
