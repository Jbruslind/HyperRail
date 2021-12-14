# Represents the current readings from the eGreenhouse package

# The time at which the readings started being taken and stopped being taken
float64 start_read_time
float64 end_read_time

# The current CO2 value (K30)
float64 co2

# The current temperature and humidity readings (SHT31)
float64 temperature
float64 humidity

# Light level readings (TSL2591)
float64 luminosity