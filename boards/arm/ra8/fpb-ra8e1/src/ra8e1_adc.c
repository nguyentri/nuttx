/****************************************************************************
 * CRITICAL FIX NEEDED: Board-level ADC initialization
 *
 * Add this function to nuttx/boards/arm/ra8/fpb-ra8e1/src/ra8e1_bringup.c
 ****************************************************************************/

#ifdef CONFIG_RA_ADC
/****************************************************************************
 * Name: ra8e1_adc_initialize
 *
 * Description:
 *   Initialize ADC for the FPB-RA8E1 board
 *
 ****************************************************************************/

int ra8e1_adc_initialize(void)
{
  FAR struct adc_dev_s *adc;
  int ret = OK;

  /* ADC0 Channel mask - Enable battery monitoring channels */
  uint32_t adc0_chanlist = (1 << 0) | (1 << 1);  /* AN000, AN001 */
  int adc0_nchannels = 2;

#ifdef CONFIG_RA_ADC1
  /* ADC1 Channel mask - Enable current sensor */
  uint32_t adc1_chanlist = (1 << 4);  /* AN104 mapped to ADC1 */
  int adc1_nchannels = 1;
#endif

  /* Initialize ADC0 */
  adc = ra8_adc_initialize(0, adc0_chanlist, adc0_nchannels);
  if (adc == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize ADC0\n");
      return -ENODEV;
    }

  /* Register ADC0 driver */
  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: adc_register ADC0 failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_RA_ADC1
  /* Initialize ADC1 */
  adc = ra8_adc_initialize(1, adc1_chanlist, adc1_nchannels);
  if (adc == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize ADC1\n");
      return -ENODEV;
    }

  /* Register ADC1 driver */
  ret = adc_register("/dev/adc1", adc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: adc_register ADC1 failed: %d\n", ret);
      return ret;
    }
#endif

  syslog(LOG_INFO, "ADC initialization complete\n");
  return ret;
}
#endif /* CONFIG_RA_ADC */
