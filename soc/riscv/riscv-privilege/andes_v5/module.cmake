
if(CONFIG_MBEDTLS)
  add_subdirectory(${ZEPHYR_BASE}/modules/source/crypto/mbedtls ${ZEPHYR_BASE}/modules/source/crypto/mbedtls)
endif()

if(CONFIG_TINYCRYPT)
  add_subdirectory(${ZEPHYR_BASE}/modules/source/crypto/tinycrypt ${ZEPHYR_BASE}/modules/source/crypto/tinycrypt)
endif()
