#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <libusb-1.0/libusb.h>

/* Derived from https://github.com/davecrump/portsdown4/tree/master/src/adf4351 */

#include "adf4351.h"

#define VENDOR_ID      (0x0456)
#define PRODUCT_ID     (0xb40d)

static struct libusb_device_handle *devh = NULL;

int main(void)
{
    uint64_t adf4351_requested_frequency = 2405.25e6;// Hz
    uint32_t adf4351_requested_power = 3; // 3 = +5 dbm

    uint32_t adf4351_requested_ref_freq = 100e6; // Hz
    uint8_t adf4351_low_spur_mode_enable = 0; // 0 = Low Noise, 1 = Low Spurs

    adf4350_param adf_parameters =
    {
      // Calculation inputs
      .clkin=adf4351_requested_ref_freq,
      .channel_spacing=5000,
      .power_up_frequency=adf4351_requested_frequency,
      .reference_div_factor=2,
      .reference_doubler_enable=0,
      .reference_div2_enable=1,

       // r2_user_settings
      .phase_detector_polarity_positive_enable=1,
      .lock_detect_precision_6ns_enable=0,
      .lock_detect_function_integer_n_enable=0,
      .charge_pump_current=2500, // uA
      .muxout_select=0,
      .low_spur_mode_enable=adf4351_low_spur_mode_enable,

      // r3_user_settings
      .cycle_slip_reduction_enable=1,
      .charge_cancellation_enable=0,
      .anti_backlash_3ns_enable=0,
      .band_select_clock_mode_high_enable=0,
      .clk_divider_12bit=0,
      .clk_divider_mode=0,

      // r4_user_settings
      .aux_output_enable=0,
      .aux_output_fundamental_enable=1,
      .mute_till_lock_enable=1,
      .output_power=adf4351_requested_power,
      .aux_output_power=0,

      .vco_powerdown = 0
    };

    adf4350_setup(&adf_parameters);

	int rc;

    rc = libusb_init(NULL);
    if (rc < 0)
    {
        fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(rc));
        exit(1);
    }

    devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!devh)
    {
        fprintf(stderr, "Error finding USB device\n");
        goto out;
    }

    rc = libusb_set_configuration(devh, 1);
    if (rc < 0) {
        fprintf(stderr, "Error setting configuration: %s\n",
                libusb_error_name(rc));
        goto out;
    }

    if (libusb_kernel_driver_active(devh, 0)) {
        libusb_detach_kernel_driver(devh, 0);
    }
    rc = libusb_claim_interface(devh, 0);
    if (rc < 0) {
        fprintf(stderr, "Error claiming interface: %s\n",
                libusb_error_name(rc));
        goto out;
    }

    uint32_t reg_write;
    unsigned char reg_bytes[4];

   	for(int i = 5; i >= 0; i--)
   	{
        reg_write = adf_parameters.state.regs[i] | i;

        printf("reg %d: 0x%08x\n", i, reg_write);

        reg_bytes[0] = (reg_write >> 24) & 0xFF;
        reg_bytes[1] = (reg_write >> 16) & 0xFF;
        reg_bytes[2] = (reg_write >> 8) & 0xFF;
        reg_bytes[3] = (reg_write >> 0) & 0xFF;

        rc = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT | LIBUSB_RECIPIENT_DEVICE, 0xDD, 0x0000, 0x0000, reg_bytes, 4, 0);
	    if (rc < 0)
        {
	        fprintf(stderr, "Error during control transfer: %s\n",
	                libusb_error_name(rc));
	    }
   	}


    unsigned char response_bytes[16];

    rc = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN | LIBUSB_RECIPIENT_INTERFACE, 0xDE, 0x0000, 0x0000, response_bytes, 16, 0);
    if (rc < 0)
    {
        fprintf(stderr, "Error during control transfer: %s\n",
                libusb_error_name(rc));
    }
    else
    {
        uint64_t device_time = ((uint64_t)response_bytes[0] << 56)
                                + ((uint64_t)response_bytes[1] << 48)
                                + ((uint64_t)response_bytes[2] << 40)
                                + ((uint64_t)response_bytes[3] << 32)
                                + (response_bytes[4] << 24)
                                + (response_bytes[5] << 16)
                                + (response_bytes[6] << 8)
                                + response_bytes[7];
        uint64_t device_lastlockloss = ((uint64_t)response_bytes[8] << 56)
                                + ((uint64_t)response_bytes[9] << 48)
                                + ((uint64_t)response_bytes[10] << 40)
                                + ((uint64_t)response_bytes[11] << 32)
                                + (response_bytes[12] << 24)
                                + (response_bytes[13] << 16)
                                + (response_bytes[14] << 8)
                                + response_bytes[15];

        printf("Device uptime:   %.3fs\n", (float)device_time/1.0e3);
        printf("Last lock time:  %.3fs\n", (float)device_lastlockloss/1.0e3);
        printf("Time since lock loss: %.3fs\n", (float)(device_time - device_lastlockloss)/1.0e3);
    }

    libusb_release_interface(devh, 0);

out:
    if (devh)
            libusb_close(devh);
    libusb_exit(NULL);
    return rc;
}