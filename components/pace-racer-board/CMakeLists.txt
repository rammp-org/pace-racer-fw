idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES adc base_component bldc_driver bldc_motor drv8353 esp_driver_spi esp_eth esp_event esp_hw_support esp_netif filters i2c interrupt led lm75adp math mt6701 pid spi timer w5500
  REQUIRED_IDF_TARGETS "esp32s3"
  )
