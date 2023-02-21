#include "stm32f4xx.h"
#include "VL53L0X.h"
#include "myiic.h"
#include "timer.h"
#include "usart.h"

/*If USE_TIMEOUT ==1, when MCU can't read the correct register value, it will try reading register repetitively while the time doesn't exceed timeout.*/
/*If USE_TIMEOUT ==0, when MCU can't read the correct register value, the program will block until the reading is successful.*/
#define USE_TIMEOUT 1 

#if USE_TIMEOUT == 1
	// Record the current time to check an upcoming timeout against
	#define startTimeout() (timeout_start_ms = runTimeTicks)

	// Check if timeout is enabled (set to nonzero value) and has expired
	#define checkTimeoutExpired() (io_timeout > 0 && (runTimeTicks - timeout_start_ms) > io_timeout)
#else
	#define startTimeout() (timeout_start_ms = 0)
	#define checkTimeoutExpired() 0
#endif


// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

uint16_t io_timeout;
uint8_t did_timeout;	//bool
uint32_t timeout_start_ms;
uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t measurement_timing_budget_us;


/*******************************************************************************

  * @name VL530LX_init 
  * @description Initialize sensor
	
  * @param io_2v8 If io_2v8 is 1, the sensor is configured for 2.8V mode
  * @param timeout If MCU can't read the correct register value, it will try 
									 reading register repetitively while the time doesn't exceed
									 timeout value. The unit is milisecond.
  * @return 1 for success, 0 for failure.
	
*******************************************************************************/
uint8_t VL530LX_init(uint8_t io_2v8, uint16_t timeout)
{
	uint8_t i;
	uint8_t spad_count;
  uint8_t spad_type_is_aperture;
	uint8_t ref_spad_map[6];
	uint8_t first_spad_to_enable;
  uint8_t spads_enabled = 0;
	
	io_timeout = timeout;
	did_timeout = 0;
	
	//your IIC initialize code
	IIC_Init();
	//code ends
	
	#if USE_TIMEOUT == 1
		//your heartbeat timer code
		TIM3_Int_Init(1000-1,84-1);
		//code ends
	#endif

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    VL53L0X_WriteReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, VL53L0X_ReadReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  VL53L0X_WriteReg(0x88, 0x00);

  VL53L0X_WriteReg(0x80, 0x01);
  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x00, 0x00);
  stop_variable = VL53L0X_ReadReg(0x91);
  VL53L0X_WriteReg(0x00, 0x01);
  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  VL53L0X_WriteReg(MSRC_CONFIG_CONTROL, VL53L0X_ReadReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  VL53L0X_SetSignalRateLimit(0.25);

  VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end


  // VL53L0X_StaticInit() begin

  if (!VL53L0X_GetSpadInfo(&spad_count, &spad_type_is_aperture)) { return 0; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  
  VL53L0X_ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  VL53L0X_WriteReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad

  for(i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  VL53L0X_WriteMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x00, 0x00);

  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x09, 0x00);
  VL53L0X_WriteReg(0x10, 0x00);
  VL53L0X_WriteReg(0x11, 0x00);

  VL53L0X_WriteReg(0x24, 0x01);
  VL53L0X_WriteReg(0x25, 0xFF);
  VL53L0X_WriteReg(0x75, 0x00);

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x4E, 0x2C);
  VL53L0X_WriteReg(0x48, 0x00);
  VL53L0X_WriteReg(0x30, 0x20);

  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x30, 0x09);
  VL53L0X_WriteReg(0x54, 0x00);
  VL53L0X_WriteReg(0x31, 0x04);
  VL53L0X_WriteReg(0x32, 0x03);
  VL53L0X_WriteReg(0x40, 0x83);
  VL53L0X_WriteReg(0x46, 0x25);
  VL53L0X_WriteReg(0x60, 0x00);
  VL53L0X_WriteReg(0x27, 0x00);
  VL53L0X_WriteReg(0x50, 0x06);
  VL53L0X_WriteReg(0x51, 0x00);
  VL53L0X_WriteReg(0x52, 0x96);
  VL53L0X_WriteReg(0x56, 0x08);
  VL53L0X_WriteReg(0x57, 0x30);
  VL53L0X_WriteReg(0x61, 0x00);
  VL53L0X_WriteReg(0x62, 0x00);
  VL53L0X_WriteReg(0x64, 0x00);
  VL53L0X_WriteReg(0x65, 0x00);
  VL53L0X_WriteReg(0x66, 0xA0);

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x22, 0x32);
  VL53L0X_WriteReg(0x47, 0x14);
  VL53L0X_WriteReg(0x49, 0xFF);
  VL53L0X_WriteReg(0x4A, 0x00);

  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x7A, 0x0A);
  VL53L0X_WriteReg(0x7B, 0x00);
  VL53L0X_WriteReg(0x78, 0x21);

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x23, 0x34);
  VL53L0X_WriteReg(0x42, 0x00);
  VL53L0X_WriteReg(0x44, 0xFF);
  VL53L0X_WriteReg(0x45, 0x26);
  VL53L0X_WriteReg(0x46, 0x05);
  VL53L0X_WriteReg(0x40, 0x40);
  VL53L0X_WriteReg(0x0E, 0x06);
  VL53L0X_WriteReg(0x20, 0x1A);
  VL53L0X_WriteReg(0x43, 0x40);

  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x34, 0x03);
  VL53L0X_WriteReg(0x35, 0x44);

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x31, 0x04);
  VL53L0X_WriteReg(0x4B, 0x09);
  VL53L0X_WriteReg(0x4C, 0x05);
  VL53L0X_WriteReg(0x4D, 0x04);

  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x44, 0x00);
  VL53L0X_WriteReg(0x45, 0x20);
  VL53L0X_WriteReg(0x47, 0x08);
  VL53L0X_WriteReg(0x48, 0x28);
  VL53L0X_WriteReg(0x67, 0x00);
  VL53L0X_WriteReg(0x70, 0x04);
  VL53L0X_WriteReg(0x71, 0x01);
  VL53L0X_WriteReg(0x72, 0xFE);
  VL53L0X_WriteReg(0x76, 0x00);
  VL53L0X_WriteReg(0x77, 0x00);

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x0D, 0x01);

  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x80, 0x01);
  VL53L0X_WriteReg(0x01, 0xF8);

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x8E, 0x01);
  VL53L0X_WriteReg(0x00, 0x01);
  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  VL53L0X_WriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  VL53L0X_WriteReg(GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_ReadReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  VL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = VL53L0X_GetMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  VL53L0X_SetMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!VL53L0X_PerformSingleRefCalibration(0x40)) { return 0; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!VL53L0X_PerformSingleRefCalibration(0x00)) { return 0; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return 1;
}


// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.

/*******************************************************************************

  * @name VL53L0X_SetSignalRateLimit
  * @description Set the return signal rate limit in units of MCPS (mega counts
								 per second).This represents the amplitude of the signal reflected
								 from the target and detected by the device.

  * @param limit_Mcps Lower the limit increases the potential range of the sensor 
											but also decrease accuracy.
  * @return 1 for success, 0 for failure.

*******************************************************************************/
uint8_t VL53L0X_SetSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0.0f || limit_Mcps > 511.99f) { return 0; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  VL53L0X_WriteReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(limit_Mcps * (1 << 7)));
  return 1;
}

float VL53L0X_GetSignalRateLimit(void)
{
  return (float)VL53L0X_ReadReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
/*******************************************************************************

  * @name VL53L0X_SetMeasurementTimingBudget
  * @description Set the measurement timing budget in microseconds, which is the
								 time allowed for one measurement.

  * @param budget_us A longer timing budget allows for more accurate measurements.
										 Defaults timing budget is 33000 microseconds, and the minimum
										 timing budget is 20000 microseconds.
  * @return 1 for success, 0 for failure.

*******************************************************************************/
uint8_t VL53L0X_SetMeasurementTimingBudget(uint32_t budget_us)
{
	uint32_t used_budget_us;
	uint32_t final_range_timeout_us;
	uint16_t final_range_timeout_mclks;
	
  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return 0; }

  used_budget_us = StartOverhead + EndOverhead;

  VL53L0X_GetSequenceStepEnables(&enables);
  VL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return 0;
    }

    final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    final_range_timeout_mclks = VL53L0X_TimeoutMicrosecondsToMclks(final_range_timeout_us,timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_WriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_EncodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return 1;
}

/*******************************************************************************

  * @name VL53L0X_GetMeasurementTimingBudget
  * @description Get the measurement timing budget.

  * @param None
  * @return Timing budget in microseconds. 

*******************************************************************************/
uint32_t VL53L0X_GetMeasurementTimingBudget(void)
{
  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  VL53L0X_GetSequenceStepEnables(&enables);
  VL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

/*******************************************************************************

  * @name VL53L0X_SetVcselPulsePeriod
  * @description Set the VCSEL (vertical cavity surface emitting laser) pulse 
								 period for the given period type to the given value in PCLKs.
								 Longer periods seem to increase the potential range of the sensor.

  * @param type VcselPeriodPreRange or VcselPeriodFinalRange.
  * @param period_pclks VcselPeriodPreRange: 12-18 (initialized default: 14)
												VcselPeriodFinalRange: 8 to 14 (initialized default: 10)
  * @return 1 for success, 0 for failure.

*******************************************************************************/
uint8_t VL53L0X_SetVcselPulsePeriod(enum vcselPeriodType type, uint8_t period_pclks)
{
	uint16_t new_pre_range_timeout_mclks;
	uint16_t new_msrc_timeout_mclks;
	uint16_t new_final_range_timeout_mclks;
	uint8_t sequence_config;
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  VL53L0X_GetSequenceStepEnables(&enables);
  VL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        VL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        VL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        VL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        VL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return 0;
    }
    VL53L0X_WriteReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    VL53L0X_WriteReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    new_pre_range_timeout_mclks = VL53L0X_TimeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    VL53L0X_WriteReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_EncodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    new_msrc_timeout_mclks = VL53L0X_TimeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    VL53L0X_WriteReg(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        VL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        VL53L0X_WriteReg(0xFF, 0x01);
        VL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x30);
        VL53L0X_WriteReg(0xFF, 0x00);
        break;

      case 10:
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        VL53L0X_WriteReg(0xFF, 0x01);
        VL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_WriteReg(0xFF, 0x00);
        break;

      case 12:
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        VL53L0X_WriteReg(0xFF, 0x01);
        VL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_WriteReg(0xFF, 0x00);
        break;

      case 14:
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_WriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_WriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        VL53L0X_WriteReg(0xFF, 0x01);
        VL53L0X_WriteReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_WriteReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return 0;
    }

    // apply new VCSEL period
    VL53L0X_WriteReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    new_final_range_timeout_mclks = VL53L0X_TimeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_WriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_EncodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return 0;
  }

  // "Finally, the timing budget must be re-applied"

  VL53L0X_SetMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  sequence_config = VL53L0X_ReadReg(SYSTEM_SEQUENCE_CONFIG);
  VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  VL53L0X_PerformSingleRefCalibration(0x0);
  VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return 1;
}
/*******************************************************************************

  * @name VL53L0X_GetVcselPulsePeriod
  * @description Return the VCSEL (vertical cavity surface emitting laser) pulse 
								 period for the given period type.

  * @param type VcselPeriodPreRange or VcselPeriodFinalRange.
  * @return Pulse period in PCLKs

*******************************************************************************/
uint8_t VL53L0X_GetVcselPulsePeriod(enum vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(VL53L0X_ReadReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(VL53L0X_ReadReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

/*******************************************************************************

  * @name VL53L0X_StartContinuous
  * @description Start continuous ranging measurements.

  * @param period_ms If period_ms is 0, the sensor takes measurements as often 
										 as possible. the period determins the interval between
										 measurements. 
  * @return None

*******************************************************************************/
void VL53L0X_StartContinuous(uint32_t period_ms)
{
  VL53L0X_WriteReg(0x80, 0x01);
  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x00, 0x00);
  VL53L0X_WriteReg(0x91, stop_variable);
  VL53L0X_WriteReg(0x00, 0x01);
  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = VL53L0X_ReadReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    VL53L0X_WriteReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    VL53L0X_WriteReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    VL53L0X_WriteReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

/*******************************************************************************

  * @name VL53L0X_StopContinuous
  * @description Stop continuous ranging measurements.

  * @param None
  * @return None

*******************************************************************************/
void VL53L0X_StopContinuous(void)
{
  VL53L0X_WriteReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x00, 0x00);
  VL53L0X_WriteReg(0x91, 0x00);
  VL53L0X_WriteReg(0x00, 0x01);
  VL53L0X_WriteReg(0xFF, 0x00);
}

// 
// (VL53L0X_ReadSingleMeasurement() also calls this function after starting a
// single-shot range measurement)

/*******************************************************************************

  * @name VL53L0X_ReadContinuousMeasurement
  * @description Reading measurement when continuous mode is active.

  * @param None
  * @return Measurement in milimeter.

*******************************************************************************/
uint16_t VL53L0X_ReadContinuousMeasurement(void)
{
	uint16_t range;
  startTimeout();
  while ((VL53L0X_ReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = 1;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  range = VL53L0X_ReadReg16Bit(RESULT_RANGE_STATUS + 10);

  VL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

/*******************************************************************************

  * @name VL53L0X_ReadSingleMeasurement
  * @description Reading measurement of a single-shot measurement.

  * @param None
  * @return Measurement in milimeter.

*******************************************************************************/
uint16_t VL53L0X_ReadSingleMeasurement(void)
{
  VL53L0X_WriteReg(POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x00, 0x00);
  VL53L0X_WriteReg(0x91, stop_variable);
  VL53L0X_WriteReg(0x00, 0x01);
  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);

  VL53L0X_WriteReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (VL53L0X_ReadReg(SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = 1;
      return 65535;
    }
  }

  return VL53L0X_ReadContinuousMeasurement();
}

/*******************************************************************************

  * @name VL53L0X_SetTimeout
  * @description Set timeout.

  * @param timeout If MCU can't read the correct register value, it will try 
									 reading register repetitively while the time doesn't exceed
									 timeout value. The unit is milisecond.
  * @return None

*******************************************************************************/
void VL53L0X_SetTimeout(uint16_t timeout)
{
	io_timeout = timeout;
}

uint16_t VL53L0X_GetTimeout(void)
{ 
	return io_timeout;
}

/*******************************************************************************

  * @name VL53L0X_TimeoutOccurred
  * @description Judge if a timeout occurs in read functions.

  * @param None
  * @return 1 for timeout occurs.

*******************************************************************************/
uint8_t VL53L0X_TimeoutOccurred(void)
{
  uint8_t tmp = did_timeout;
  did_timeout = 0;
  return tmp;
}


/*******************************************************************************

  * @name VL53L0X_GetSpadInfo
  * @description Get reference SPAD(single photon avalanche diode) count and type.

  * @param count Pointer to SPAD count.
  * @param type_is_aperture Pointer to SPAD type.
  * @return 1 for success, 0 for failure.

*******************************************************************************/
uint8_t VL53L0X_GetSpadInfo(uint8_t * count, uint8_t * type_is_aperture)
{
  uint8_t tmp;

  VL53L0X_WriteReg(POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x00, 0x00);

  VL53L0X_WriteReg(0xFF, 0x06);
  VL53L0X_WriteReg(0x83, VL53L0X_ReadReg(0x83) | 0x04);
  VL53L0X_WriteReg(0xFF, 0x07);
  VL53L0X_WriteReg(0x81, 0x01);

  VL53L0X_WriteReg(POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);

  VL53L0X_WriteReg(0x94, 0x6b);
  VL53L0X_WriteReg(0x83, 0x00);
  startTimeout();
  while (VL53L0X_ReadReg(0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return 0; }
  }
  VL53L0X_WriteReg(0x83, 0x01);
  tmp = VL53L0X_ReadReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  VL53L0X_WriteReg(0x81, 0x00);
  VL53L0X_WriteReg(0xFF, 0x06);
  VL53L0X_WriteReg(0x83, VL53L0X_ReadReg( 0x83  & ~0x04));
  VL53L0X_WriteReg(0xFF, 0x01);
  VL53L0X_WriteReg(0x00, 0x01);

  VL53L0X_WriteReg(0xFF, 0x00);
  VL53L0X_WriteReg(POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);

  return 1;
}

/*******************************************************************************

  * @name VL53L0X_GetSequenceStepEnables
  * @description Get sequence step enables

  * @param enables Pointer to struct SequenceStepEnables.
  * @return None

*******************************************************************************/
void VL53L0X_GetSequenceStepEnables(struct SequenceStepEnables * enables)
{
  uint8_t sequence_config = VL53L0X_ReadReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

/*******************************************************************************

  * @name VL53L0X_GetSequenceStepTimeouts
  * @description Get sequence step timeouts, including all timeouts instead of 
								 just the requested one, and also stores intermediate values.

  * @param enables Pointer to struct SequenceStepEnables.
  * @param timeouts Pointer to struct SequenceStepTimeouts.
  * @return None

*******************************************************************************/
void VL53L0X_GetSequenceStepTimeouts(struct SequenceStepEnables const * enables, struct SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = VL53L0X_ReadReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us = VL53L0X_TimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks = VL53L0X_DecodeTimeout(VL53L0X_ReadReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us = VL53L0X_TimeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks = VL53L0X_DecodeTimeout(VL53L0X_ReadReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us = VL53L0X_TimeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

/*******************************************************************************

  * @name VL53L0X_PerformSingleRefCalibration
  * @description Calibration.

  * @param vhv_init_byte
  * @return 1 for success, 0 for failure.

*******************************************************************************/
uint8_t VL53L0X_PerformSingleRefCalibration(uint8_t vhv_init_byte)
{
  VL53L0X_WriteReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((VL53L0X_ReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return 0; }
  }

  VL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  VL53L0X_WriteReg(SYSRANGE_START, 0x00);

  return 1;
}

// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
/*******************************************************************************

  * @name VL53L0X_DecodeTimeout
  * @description Decode sequence step timeout in MCLKs from register value.

  * @param reg_val The value of one register.
  * @return Decoded timeout value in milisecond.

*******************************************************************************/
uint16_t VL53L0X_DecodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

/*******************************************************************************

  * @name VL53L0X_EncodeTimeout
  * @description Encode sequence step timeout register value from timeout in MCLKs.
								 The encoded value of timeout will be written into register.

  * @param timeout_mclks
  * @return Encoded timeout value.

*******************************************************************************/
uint16_t VL53L0X_EncodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// 
// based on VL53L0X_calc_timeout_us()
/*******************************************************************************

  * @name VL53L0X_TimeoutMclksToMicroseconds
  * @description Convert sequence step timeout from MCLKs to microseconds with 
								 given VCSEL period in PCLKs.

  * @param timeout_period_mclks
  * @param vcsel_period_pclks
  * @return Converted timeout

*******************************************************************************/
uint32_t VL53L0X_TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// 
// based on VL53L0X_calc_timeout_mclks()
/*******************************************************************************

  * @name VL53L0X_TimeoutMicrosecondsToMclks
  * @description Convert sequence step timeout from microseconds to MCLKs with 
								 given VCSEL period in PCLKs.

  * @param timeout_period_us
  * @param vcsel_period_pclks
  * @return MCLKs

*******************************************************************************/
uint32_t VL53L0X_TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

/*******************************************************************************

  * @name VL53L0X_WriteReg
  * @description Write an 8-bit data to register.

  * @param reg register to be written.
  * @param data 8-bit data to be written.
  * @return None

*******************************************************************************/
void VL53L0X_WriteReg(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(data);
	IIC_ReceiveAck();		//waiting for ack
	
  IIC_Stop();
}

/*******************************************************************************

  * @name VL53L0X_WriteReg16Bit
  * @description Write an 16-bit data to register.

  * @param reg register to be written.
  * @param data 16-bit data to be written.
  * @return None

*******************************************************************************/
void VL53L0X_WriteReg16Bit(uint8_t reg, uint16_t data)
{
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);	//0 means write
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte((data>>8) & 0xFF);
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(data & 0xFF);
	IIC_ReceiveAck();		//waiting for ack
	 
  IIC_Stop();
}

/*******************************************************************************

  * @name VL53L0X_WriteReg32Bit
  * @description Write an 32-bit data to register.

  * @param reg register to be written.
  * @param data 32-bit data to be written.
  * @return None

*******************************************************************************/
void VL53L0X_WriteReg32Bit(uint8_t reg, uint32_t data)
{
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);	//0 means write
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte((data>>24) & 0xFF);
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte((data>>16) & 0xFF);
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte((data>>8) & 0xFF);
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(data & 0xFF);
	IIC_ReceiveAck();		//waiting for ack
	 
  IIC_Stop();
}

/*******************************************************************************

  * @name VL53L0X_ReadReg
  * @description Read an 8-bit data from register.

  * @param reg register to be read.
  * @return 8-bit data.

*******************************************************************************/
uint8_t VL53L0X_ReadReg(uint8_t reg)
{
  uint8_t res;
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);	//0 means write
	IIC_ReceiveAck();		//waiting for ack

	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_READ);	//1 means read
	IIC_ReceiveAck();		//waiting for ack
	
	res = IIC_ReceiveByte();
	IIC_SendAck(1);  //noack
	
	IIC_Stop();
	return res;
}

/*******************************************************************************

  * @name VL53L0X_ReadReg16Bit
  * @description Read an 16-bit data from register.

  * @param reg register to be read.
  * @return 16-bit data.

*******************************************************************************/
uint16_t VL53L0X_ReadReg16Bit(uint8_t reg)
{
  uint16_t res;
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);	//0 means write
	IIC_ReceiveAck();		//waiting for ack

	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_READ);
	IIC_ReceiveAck();		//waiting for ack
	
	res = (uint16_t)IIC_ReceiveByte() << 8;
	IIC_SendAck(0);  //ack
	
	res |= IIC_ReceiveByte();
	IIC_SendAck(1);  //no ack
	
	IIC_Stop();
	return res;
}

/*******************************************************************************

  * @name VL53L0X_ReadReg32Bit
  * @description Read an 32-bit data from register.

  * @param reg register to be read.
  * @return 32-bit data.

*******************************************************************************/
uint32_t VL53L0X_ReadReg32Bit(uint8_t reg)
{
	uint32_t res;
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);	//0 means write
	IIC_ReceiveAck();		//waiting for ack

	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_READ);	//1 means read
	IIC_ReceiveAck();		//waiting for ack
	
	res = (uint32_t)IIC_ReceiveByte() << 24;
	IIC_SendAck(0);  //ack
	
	res |= (uint32_t)IIC_ReceiveByte() << 16;
	IIC_SendAck(0);  //ack
	
	res |= (uint32_t)IIC_ReceiveByte() << 8;
	IIC_SendAck(0);  //ack
	
	res |= IIC_ReceiveByte();
	IIC_SendAck(1);  //no ack
	
	IIC_Stop();
	return res;
}

/*******************************************************************************

  * @name VL53L0X_WriteMulti
  * @description Write some bytes from the given array to the sensor.

  * @param reg Register to be written.
  * @param src Pointor to the array.
	* @param count Amounts of 8-bit data to be written.
  * @return None

*******************************************************************************/
void VL53L0X_WriteMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
	uint8_t i;
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);	//0 means write
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack

  for(i = 0; i<count; i++)
  {
		IIC_SendByte(*(src+i));	
		IIC_ReceiveAck();
  }

	IIC_Stop();
}

// d an arbitrary number of bytes from the , starting at the given
// register, into the given array
/*******************************************************************************

  * @name VL53L0X_ReadMulti
  * @description Read some bytes from the sensor at the given given register, into 
								 the given array.

  * @param reg Register to be read.
  * @param dst Pointor to the array.
	* @param count Amounts of 8-bit data to be read.
  * @return None

*******************************************************************************/
void VL53L0X_ReadMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{
	uint8_t i;
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_WRITE);	//0 means write
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_SendByte(reg);	//sensor address
	IIC_ReceiveAck();		//waiting for ack
	
	IIC_Start();
	IIC_SendByte(VL53L0X_ADDR_READ);	//1 means read
	IIC_ReceiveAck();		//waiting for ack
	
	for(i = 0; i<count; i++)
  {
		*(dst+i) = IIC_ReceiveByte();
		IIC_SendAck(0);		//ack
  }
	
  IIC_Stop();
}

