/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_app_examples.c
 *
 * Unified example wrappers that provide a single entry point
 * ra8e1_<app>_example() for each board example to simplify calls
 * from ra8e1_bringup().
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>

#include "fpb-ra8e1.h"

#ifdef CONFIG_RA8E1_SPI_LOOPBACK_EXAMPLE
int ra8e1_spi_loopback_example(void)
{
  int ret = ra8e1_spi_loopback_init();
  if (ret < 0)
    {
      syslog(LOG_INFO, "Test initialization failed: %d\n", ret);
      return ret;
    }

  return ra8e1_spi_loopback_main(0, NULL);
}
#endif

#ifdef CONFIG_RA8E1_SPI_MASTERSLAVE_EXAMPLE
int ra8e1_spi_masterslave_example(void)
{
  int ret = ra8e1_spi_masterslave_init();
  if (ret < 0)
    {
      return ret;
    }

  return ra8e1_spi_masterslave_test();
}
#endif

#ifdef CONFIG_RA_ADC_BMS_EXAMPLE
int ra8e1_adc_bms_example(void)
{
  int ret = ra8e1_adc_bms_init();
  if (ret < 0)
    {
      return ret;
    }

  return ra8e1_adc_bms_main(0, NULL);
}
#endif

#ifdef CONFIG_RA_CODE_FLASH_EXAMPLE
int ra8e1_code_flash_example(void)
{
  int ret = ra8e1_code_flash_init();
  if (ret < 0)
    {
      return ret;
    }

  /* Run any self-tests if available */
  ra8e1_code_flash_test();
  return 0;
}
#endif

#ifdef CONFIG_RA_DATA_FLASH_EXAMPLE
int ra8e1_data_flash_example(void)
{
  int ret = ra8e1_data_flash_init();
  if (ret < 0)
    {
      return ret;
    }

  /* Run test routine if present */
  ra8e1_data_flash_test();
  return 0;
}
#endif

#ifdef CONFIG_RA8E1_PWM_ESCS_EXAMPLE
int ra8e1_gpt_escs_example(void)
{
  int ret = ra8e1_gpt_escs_init();
  if (ret < 0)
    {
      return ret;
    }

  return ra8e1_gpt_escs_main(0, NULL);
}
#endif

#ifdef CONFIG_RA8E1_GPS_EXAMPLE
int ra8e1_gps_example(void)
{
  int ret = ra8e1_gps_init();
  if (ret < 0)
    {
      return ret;
    }

  return ra8e1_gps_main(0, NULL);
}
#endif

#ifdef CONFIG_RA8E1_SBUS_EXAMPLE
int ra8e1_sbus_example(void)
{
  int ret = ra8e1_sbus_init();
  if (ret < 0)
    {
      return ret;
    }

  return ra8e1_sbus_main(0, NULL);
}
#endif

#ifdef CONFIG_RA8E1_I2C_ACC_EXAMPLE
int ra8e1_i2c_acc_example(void)
{
  int ret = ra8e1_i2c_acc_init();
  if (ret < 0)
    {
      return ret;
    }

  return ra8e1_i2c_acc_main(0, NULL);
}
#endif

#ifdef CONFIG_RA8E1_I2C_GY912_EXAMPLE
int ra8e1_i2c_gy912_example(void)
{
  int ret = ra8e1_i2c_gy912_init();
  if (ret < 0)
    {
      return ret;
    }

  return ra8e1_i2c_gy912_main(0, NULL);
}
#endif

#ifdef CONFIG_RA8E1_RUST_EXAMPLE
int ra8e1_rust_example(void)
{
  /* Existing thread init is the entry point */
  return ra8e1_rust_sample_init();
}
#endif


/* Run all enabled application examples. Returns 0 on success or the
 * last non-zero error code from any example.
 */
int ra8e1_app_examples(void)
{
  int ret = 0;
  int last_err = 0;

#ifdef CONFIG_RA_GPT_PWM_EXAMPLE
  ret = ra8e1_gpt_test_main(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "GPT test failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_ADC_BMS_EXAMPLE
  ret = ra8e1_adc_bms_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ADC BMS example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_CODE_FLASH_EXAMPLE
  ret = ra8e1_code_flash_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Code Flash example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_DATA_FLASH_EXAMPLE
  ret = ra8e1_data_flash_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Data Flash example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_PWM_ESCS_EXAMPLE
  ret = ra8e1_gpt_escs_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ESCs example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_GPS_EXAMPLE
  ret = ra8e1_gps_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "GPS example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_SBUS_EXAMPLE
  ret = ra8e1_sbus_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SBUS example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_I2C_GY912_EXAMPLE
  ret = ra8e1_i2c_gy912_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "I2C GY-912 example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_I2C_ACC_EXAMPLE
  ret = ra8e1_i2c_acc_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "I2C ACC example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_SPI_LOOPBACK_EXAMPLE
  ret = ra8e1_spi_loopback_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI Loopback example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_SPI_MASTERSLAVE_EXAMPLE
  ret = ra8e1_spi_masterslave_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI Master/Slave example failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_SPI_GY912_EXAMPLE
  ret = ra8e1_spi_gy912_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI GY-912 init failed: %d\n", ret);
      last_err = ret;
    }
#endif

#ifdef CONFIG_RA8E1_RUST_EXAMPLE
  ret = ra8e1_rust_example();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Rust example failed: %d\n", ret);
      last_err = ret;
    }
#endif

  return last_err;
}
