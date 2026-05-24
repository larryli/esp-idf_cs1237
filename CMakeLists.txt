if(${IDF_VERSION_MAJOR} LESS 5 OR 
  (${IDF_VERSION_MAJOR} EQUAL 5 AND ${IDF_VERSION_MINOR} LESS_EQUAL 2))
set(REQS driver)
else()
set(REQS esp_driver_gpio)
endif()

idf_component_register(SRCS "src/cs1237.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${REQS} esp_timer)
