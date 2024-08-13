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
