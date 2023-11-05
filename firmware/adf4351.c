/***************************************************************************//**
 *   @file   adf4350.c
 *   @brief  Implementation of ADF4350 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *
********************************************************************************
 * Copyright 2012-2015(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*******************************************************************************/

#include "adf4351.h"

/***************************************************************************//**
 * @brief Increases the R counter value until the ADF4350_MAX_FREQ_PFD is
 *        greater than PFD frequency.
 *
 * @param st    - The selected structure.
 * @param r_cnt - Initial r_cnt value.
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
static int32_t adf4350_tune_r_cnt(struct adf4350_state *st, uint16_t r_cnt)
{
	struct adf4350_platform_data *pdata = st->pdata;

	st->fpfd = (st->clkin * (pdata->ref_doubler_en ? 2 : 1)) /
		   (r_cnt * (pdata->ref_div2_en ? 2 : 1));
	while (st->fpfd > ADF4350_MAX_FREQ_PFD) 
	{
		r_cnt++;
		st->fpfd = (st->clkin * (pdata->ref_doubler_en ? 2 : 1)) /
			   (r_cnt * (pdata->ref_div2_en ? 2 : 1));
	}

	return r_cnt;
}

/***************************************************************************//**
 * @brief Sets the ADF4350 frequency.
 *
 * @param st   - The selected structure.
 * @param freq - The desired frequency value.
 *
 * @return calculatedFrequency - The actual frequency value that was set.
*******************************************************************************/
static int64_t adf4350_set_freq(struct adf4350_state *st, uint64_t freq)
{
	struct adf4350_platform_data *pdata = st->pdata;
	uint64_t tmp;
	//uint32_t div_gcd;
	uint32_t prescaler;
	//uint32_t chspc;
	//uint16_t mdiv;
	uint16_t r_cnt = 0;
	uint8_t band_sel_div;

	if ((freq > ADF4350_MAX_OUT_FREQ) || (freq < ADF4350_MIN_OUT_FREQ))
		return -1;

	if (freq > ADF4350_MAX_FREQ_45_PRESC)           // 3.6 GHz
    {
		prescaler = ADF4350_REG1_PRESCALER;         // Prescaler 8/9
		//mdiv = 75;
	} 
	else
	{
		prescaler = 0;                              // Prescaler 4/5
		//mdiv = 23;
	}

	st->r4_rf_div_sel = 0;

	while (freq < ADF4350_MIN_VCO_FREQ)             // 2.2 GHz
	{
		freq <<= 1;
		st->r4_rf_div_sel++;
	}

	/*
	 * Allow a predefined reference division factor
	 * if not set, compute our own
	 */

	if (pdata->ref_div_factor)
  {
		r_cnt = pdata->ref_div_factor;
  }

	// Set ref_div to derive valid pfd
	r_cnt = adf4350_tune_r_cnt(st, r_cnt);
	// st->fpfd

	while(((int64_t)(st->r0_int + 1) * st->fpfd) < freq) {
		st->r0_int++;
	}

	int64_t remainder = freq - ((int64_t)st->r0_int * st->fpfd);

	st->r1_mod = st->fpfd / st->chspc;

	// Find valid MOD
	while(st->r1_mod > ADF4350_MAX_MODULUS)
	{
		st->r1_mod /= 2;
	}

	st->r0_fract = remainder / (st->fpfd / st->r1_mod);

	int32_t n1 = st->r0_fract, n2 = st->r1_mod;

	// Find GCD
  while(n1 != n2)
  {
      if(n1 > n2)
    	{
    		n1 -= n2;
    	}
    	else
    	{
    		n2 -= n1;
    	}
  }
  if(n1 > 1)
  {
  	st->r0_fract /= n1;
  	st->r1_mod /= n1;
  }

	band_sel_div = st->fpfd % ADF4350_MAX_BANDSEL_CLK > ADF4350_MAX_BANDSEL_CLK / 2 ?
					st->fpfd / ADF4350_MAX_BANDSEL_CLK + 1 :
					st->fpfd / ADF4350_MAX_BANDSEL_CLK;


/*
  // Channel Spacing
	chspc = st->chspc;

	do
	{
		do
		{
			do
			{
				r_cnt = adf4350_tune_r_cnt(st, r_cnt);
						printf(" st->fpfd / chspc : %d / %d\n", st->fpfd, chspc);
						st->r1_mod = st->fpfd / chspc;
						if (r_cnt > ADF4350_MAX_R_CNT)
						{
							// try higher spacing values
							chspc++;
							r_cnt = 0;
						}
						printf("r1_mod: %d, MAXmod: %d\n", st->r1_mod, ADF4350_MAX_MODULUS);
			} while ((st->r1_mod > ADF4350_MAX_MODULUS) && r_cnt);
		} while (r_cnt == 0);


		tmp = freq * (uint64_t)st->r1_mod + (st->fpfd > 1);

		tmp = (tmp / st->fpfd);	// Div round closest (n + d/2)/d

		st->r0_fract = tmp % st->r1_mod;
		tmp = tmp / st->r1_mod;

		st->r0_int = tmp;

	} while (mdiv > st->r0_int);

	band_sel_div = st->fpfd % ADF4350_MAX_BANDSEL_CLK > ADF4350_MAX_BANDSEL_CLK / 2 ?
					st->fpfd / ADF4350_MAX_BANDSEL_CLK + 1 :
					st->fpfd / ADF4350_MAX_BANDSEL_CLK;

	if (st->r0_fract && st->r1_mod)
	{
		div_gcd = gcd(st->r1_mod, st->r0_fract);
		st->r1_mod /= div_gcd;
		st->r0_fract /= div_gcd;
	}
	else
	{
		st->r0_fract = 0;
		// st->r1_mod = 1;
		st->r1_mod = 2;  // changed to agree with the AD PC application.  Makes little difference
	}
*/
	st->regs[ADF4350_REG0] = ADF4350_REG0_INT(st->r0_int) |
				 ADF4350_REG0_FRACT(st->r0_fract);

	st->regs[ADF4350_REG1] = ADF4350_REG1_PHASE(1) |
				 ADF4350_REG1_MOD(st->r1_mod) |
				 prescaler;

	st->regs[ADF4350_REG2] =
		ADF4350_REG2_10BIT_R_CNT(r_cnt) |
		ADF4350_REG2_DOUBLE_BUFF_EN |
		(pdata->ref_doubler_en ? ADF4350_REG2_RMULT2_EN : 0) |
		(pdata->ref_div2_en ? ADF4350_REG2_RDIV2_EN : 0) |
		(pdata->r2_user_settings & (ADF4350_REG2_PD_POLARITY_POS |
		ADF4350_REG2_LDP_6ns | ADF4350_REG2_LDF_INT_N |
		ADF4350_REG2_CHARGE_PUMP_CURR_uA(5000) |
		ADF4350_REG2_MUXOUT(0x7) | ADF4350_REG2_NOISE_MODE(0x0)));
//		ADF4350_REG2_MUXOUT(0x7) | ADF4350_REG2_NOISE_MODE(0x9)));

	st->regs[ADF4350_REG3] = pdata->r3_user_settings &
				 (ADF4350_REG3_12BIT_CLKDIV(0xFFF) |
				 ADF4350_REG3_12BIT_CLKDIV_MODE(0x3) |
				 ADF4350_REG3_12BIT_CSR_EN);

	st->regs[ADF4350_REG4] =
		ADF4350_REG4_FEEDBACK_FUND |
		ADF4350_REG4_RF_DIV_SEL(st->r4_rf_div_sel) |
		ADF4350_REG4_8BIT_BAND_SEL_CLKDIV(band_sel_div) |
		ADF4350_REG4_RF_OUT_EN |
		(pdata->r4_user_settings &
		(ADF4350_REG4_OUTPUT_PWR(0x3) |
		ADF4350_REG4_AUX_OUTPUT_PWR(0x3) |
		ADF4350_REG4_AUX_OUTPUT_EN |
		ADF4350_REG4_AUX_OUTPUT_FUND |
		ADF4350_REG4_MUTE_TILL_LOCK_EN));

	st->regs[ADF4350_REG5] = ADF4350_REG5_LD_PIN_MODE_DIGITAL | 0x00180000;

    tmp = (uint64_t)((st->r0_int * st->r1_mod) + st->r0_fract) * (uint64_t)st->fpfd;
    tmp = tmp / ((uint64_t)st->r1_mod * ((uint64_t)1 << st->r4_rf_div_sel));

    //printf("mdiv = %d\n", mdiv);
    //printf("r0_int = %d  r0_fract = %d  r1_mod =  %d \n", st->r0_int, st->r0_fract, st->r1_mod);
    //printf("fpfd = %d, rf_div_sel = %d\n", st->fpfd, st->r4_rf_div_sel);

	return tmp;
}

/***************************************************************************//**
 * @brief Initializes the ADF4350.
 *
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
int32_t adf4350_setup(adf4350_param *param_ptr)
{
	struct adf4350_state *st = &(param_ptr->state);

	// TODO: Remove this quick-fix
	st->pdata = &(st->_pdata);

	st->pdata->clkin = param_ptr->clkin;

	st->pdata->channel_spacing = param_ptr->channel_spacing;

	st->pdata->power_up_frequency = param_ptr->power_up_frequency;
	st->pdata->ref_div_factor = param_ptr->reference_div_factor;
	st->pdata->ref_doubler_en = param_ptr->reference_doubler_enable;
	st->pdata->ref_div2_en = param_ptr->reference_div2_enable;

	/* r2_user_settings */

	st->pdata->r2_user_settings = param_ptr->phase_detector_polarity_positive_enable ?
			ADF4350_REG2_PD_POLARITY_POS : 0;
	st->pdata->r2_user_settings |= param_ptr->lock_detect_precision_6ns_enable ?
			ADF4350_REG2_LDP_6ns : 0;
	st->pdata->r2_user_settings |= param_ptr->lock_detect_function_integer_n_enable ?
			ADF4350_REG2_LDF_INT_N : 0;
	st->pdata->r2_user_settings |= ADF4350_REG2_CHARGE_PUMP_CURR_uA(param_ptr->charge_pump_current);
	st->pdata->r2_user_settings |= ADF4350_REG2_MUXOUT(param_ptr->muxout_select);
	st->pdata->r2_user_settings |= param_ptr->low_spur_mode_enable ? ADF4350_REG2_NOISE_MODE(0x3) : 0;

	/* r3_user_settings */

	st->pdata->r3_user_settings = param_ptr->cycle_slip_reduction_enable ?
			ADF4350_REG3_12BIT_CSR_EN : 0;
	st->pdata->r3_user_settings |= param_ptr->charge_cancellation_enable ?
			ADF4351_REG3_CHARGE_CANCELLATION_EN : 0;
	st->pdata->r3_user_settings |= param_ptr->anti_backlash_3ns_enable ?
			ADF4351_REG3_ANTI_BACKLASH_3ns_EN : 0;
	st->pdata->r3_user_settings |= param_ptr->band_select_clock_mode_high_enable ?
			ADF4351_REG3_BAND_SEL_CLOCK_MODE_HIGH : 0;
	st->pdata->r3_user_settings |= ADF4350_REG3_12BIT_CLKDIV(param_ptr->clk_divider_12bit);
	st->pdata->r3_user_settings |= ADF4350_REG3_12BIT_CLKDIV_MODE(param_ptr->clk_divider_mode);

	/* r4_user_settings */

	st->pdata->r4_user_settings = param_ptr->aux_output_enable ?
			ADF4350_REG4_AUX_OUTPUT_EN : 0;
	st->pdata->r4_user_settings |= param_ptr->aux_output_fundamental_enable ?
			ADF4350_REG4_AUX_OUTPUT_FUND : 0;
	st->pdata->r4_user_settings |= param_ptr->mute_till_lock_enable ?
			ADF4350_REG4_MUTE_TILL_LOCK_EN : 0;
	st->pdata->r4_user_settings |= ADF4350_REG4_OUTPUT_PWR(param_ptr->output_power);
	st->pdata->r4_user_settings |= ADF4350_REG4_AUX_OUTPUT_PWR(param_ptr->aux_output_power);

	/* Store reference frequency */
	param_ptr->state.clkin = st->pdata->clkin;

	/* Store channel spacing */
	param_ptr->state.chspc = st->pdata->channel_spacing;

	/* Set Frequency */
	adf4350_set_freq(&(param_ptr->state), st->pdata->power_up_frequency);

	/* Set VCO Power down */
	if(param_ptr->vco_powerdown == 1)
	{
		param_ptr->state.regs[ADF4350_REG2] |= ADF4350_REG2_POWER_DOWN_EN;
	}
	else if(param_ptr->vco_powerdown == 0)
	{
		param_ptr->state.regs[ADF4350_REG2] &= ~ADF4350_REG2_POWER_DOWN_EN;
	}

  return 0;
}
