INCLUDES += -I$(STM32_MAGIC_DIR)/src

# Basic peripherals
CXXFILES += $(STM32_MAGIC_DIR)/src/Exti.cpp
CXXFILES += $(STM32_MAGIC_DIR)/src/Spi.cpp
CXXFILES += $(STM32_MAGIC_DIR)/src/Sd.cpp
CXXFILES += $(STM32_MAGIC_DIR)/src/UsartBase.cpp
CXXFILES += $(STM32_MAGIC_DIR)/src/Usart.cpp
CXXFILES += $(STM32_MAGIC_DIR)/src/UsartBuffered.cpp

# Radio modules
CXXFILES += $(STM32_MAGIC_DIR)/src/Radio/Nrf24.cpp

# For Leds
CXXFILES += $(STM32_MAGIC_DIR)/src/Led/Charlieplexing.cpp
