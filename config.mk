INCLUDES += -I$(STM32_MY_PLATFORM_DIR)/src

# Basic peripherals
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/Exti.cpp
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/Spi.cpp
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/Sd.cpp
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/UsartBase.cpp
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/Usart.cpp
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/UsartBuffered.cpp

# Radio modules
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/Radio/Nrf24.cpp

# For Leds
CXXFILES += $(STM32_MY_PLATFORM_DIR)/src/Platform/Led/Charlieplexing.cpp
