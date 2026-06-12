idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES adc base_component bldc_driver bldc_motor drv8353 esp_driver_spi filters i2c interrupt led lm75adp math mt6701 pid spi timer
  REQUIRED_IDF_TARGETS "esp32s3"
  )
