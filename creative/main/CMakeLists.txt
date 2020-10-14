set(srcs "sh1106.c" "creative.c" "dht11.c" "beep.c" "adxl345.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ".")
                    

