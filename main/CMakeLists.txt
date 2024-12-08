idf_component_register(SRCS "driver/drv_bmp180.c" 

                            "bsp/bsp_i2c.c" 
                            "bsp/bsp_timer.c" 
                            
                            "main.c"
                            
                    INCLUDE_DIRS "." "bsp" "driver")