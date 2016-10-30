MCU_MODEL = HD
HSE_CLOCK = 16000000
SYSTEM_CLOCK = 72000000

PROJECT_SOURCES = main.c \
	exti.c \
	printf_config.c \
	\
	engines/encoders.c \
	engines/engines.c \
	engines/engines_bridge.c \
	engines/engines_pwm.c \
	\
	lights/lights.c \
	\
	power/power.c \
	power/power_adc.c \
	power/power_led.c \
	\
	rosov/bumper.c \
	rosov/comm.c \
	\
	video/video.c \
	video/video_tx.c
