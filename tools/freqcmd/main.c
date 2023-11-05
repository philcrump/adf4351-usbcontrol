#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <libusb-1.0/libusb.h>

#define VENDOR_ID      (0x0456)
#define PRODUCT_ID     (0xb40d)

static struct libusb_device_handle *devh = NULL;

int main(void)
{
    uint64_t adf4351_requested_frequency = 2405e6;// Hz

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

    unsigned char freq_bytes[8];

    freq_bytes[0] = (adf4351_requested_frequency >> 56) & 0xFF;
    freq_bytes[1] = (adf4351_requested_frequency >> 48) & 0xFF;
    freq_bytes[2] = (adf4351_requested_frequency >> 40) & 0xFF;
    freq_bytes[3] = (adf4351_requested_frequency >> 32) & 0xFF;
    freq_bytes[4] = (adf4351_requested_frequency >> 24) & 0xFF;
    freq_bytes[5] = (adf4351_requested_frequency >> 16) & 0xFF;
    freq_bytes[6] = (adf4351_requested_frequency >> 8) & 0xFF;
    freq_bytes[7] = (adf4351_requested_frequency >> 0) & 0xFF;

    rc = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT | LIBUSB_RECIPIENT_DEVICE, 0xDC, 0x0000, 0x0000, freq_bytes, 8, 0);
    if (rc < 0)
    {
        fprintf(stderr, "Error during control transfer: %s\n",
                libusb_error_name(rc));
    }

    libusb_release_interface(devh, 0);

out:
    if (devh)
            libusb_close(devh);
    libusb_exit(NULL);
    return rc;
}