F3_TARGETS  += $(TARGET)

FEATURES  = VCP 

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c

ifeq ($(TARGET), CRAZYBEEF3FS)
TARGET_SRC += \
            drivers/rx/rx_a7105.c \
            rx/flysky.c
else
ifeq ($(TARGET), CRAZYBEEF3FR)
TARGET_SRC += \
            drivers/rx/rx_cc2500.c \
            rx/cc2500_frsky_shared.c \
            rx/cc2500_frsky_d.c \
            rx/cc2500_frsky_x.c
endif
endif
