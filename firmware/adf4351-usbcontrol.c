#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>

#define USB_VENDOR_ENDPOINT_OUT         0x40
#define USB_VENDOR_ENDPOINT_IN          0xC1

const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0456,
    .idProduct = 0xb40d,
    .bcdDevice = 0x0000,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};

const struct usb_interface_descriptor iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 0,
    .bInterfaceClass = 0xFF,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
};

const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = &iface,
}};

const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

const char *usb_strings[] = {
    "ANALOG DEVICES",
    "ADF4xxx USB Eval Board"
};

static volatile uint64_t time_millis = 0;
static volatile uint64_t last_lostlock_millis = 0;

uint32_t usbd_control_buffer[32];

static uint32_t reg = 0;
static uint8_t response_buf[24];

static enum usbd_request_return_codes vendor_control_callback(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)buf;
    (void)len;
    (void)complete;
    (void)usbd_dev;

    switch (req->bmRequestType)
    {
        case USB_VENDOR_ENDPOINT_OUT:

            switch (req->bRequest)
            {
                case 0xDD:
                {
                    if (*len != 4)
                        return USBD_REQ_NOTSUPP;

                    reg = ((*buf)[0] << 24) | ((*buf)[1] << 16) |
                        ((*buf)[2] << 8) | (*buf)[3];

                    int i, k;

                    gpio_clear(GPIOB, GPIO13); // LE

                    gpio_clear(GPIOB, GPIO15); // Clk

                    for(k = 0; k < 4; k++)
                    {
                        for(i = 7; i >= 0; i--)
                        {
                            if((reg >> ((k*8)+i)) & 0x1)
                            {
                                gpio_set(GPIOB, GPIO14); // Data
                            }
                            else
                            {
                                gpio_clear(GPIOB, GPIO14); // Data
                            }

                            gpio_set(GPIOB, GPIO15); // Clk

                            gpio_clear(GPIOB, GPIO15); // Clk
                        }
                    }

                    gpio_set(GPIOB, GPIO13); // LE

                    // no data in response
                    *buf = (void *)0;
                    *len = 0;

                    return USBD_REQ_HANDLED;
                }
                break;

                default:
                    return USBD_REQ_NOTSUPP;
            }
            break;

        case USB_VENDOR_ENDPOINT_IN:
            switch (req->bRequest)
            {
                case 0xDE:
                {
                    response_buf[0] = (time_millis >> 56) & 0xFF;
                    response_buf[1] = (time_millis >> 48) & 0xFF;
                    response_buf[2] = (time_millis >> 40) & 0xFF;
                    response_buf[3] = (time_millis >> 32) & 0xFF;
                    response_buf[4] = (time_millis >> 24) & 0xFF;
                    response_buf[5] = (time_millis >> 16) & 0xFF;
                    response_buf[6] = (time_millis >> 8) & 0xFF;
                    response_buf[7] = (time_millis >> 0) & 0xFF;

                    response_buf[8] = (last_lostlock_millis >> 56) & 0xFF;
                    response_buf[9] = (last_lostlock_millis >> 48) & 0xFF;
                    response_buf[10] = (last_lostlock_millis >> 40) & 0xFF;
                    response_buf[11] = (last_lostlock_millis >> 32) & 0xFF;
                    response_buf[12] = (last_lostlock_millis >> 24) & 0xFF;
                    response_buf[13] = (last_lostlock_millis >> 16) & 0xFF;
                    response_buf[14] = (last_lostlock_millis >> 8) & 0xFF;
                    response_buf[15] = (last_lostlock_millis >> 0) & 0xFF;

                    *buf = response_buf;
                    *len = 16;

                    return USBD_REQ_HANDLED;
                }
                break;

                default:
                    return USBD_REQ_NOTSUPP;
            }
            break;

        default:
            return USBD_REQ_NOTSUPP;
    }
}

static void usb_set_config_cb(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;
    usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_VENDOR,
        USB_REQ_TYPE_TYPE, vendor_control_callback);
}

void sys_tick_handler(void)
{
    time_millis++;

    if(0 == gpio_get(GPIOA, GPIO8))
    {
        last_lostlock_millis = time_millis;
    }
}

int main(void)
{
    /* Clock setup */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    /* Systick */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO14);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);

    gpio_clear(GPIOB, GPIO13); // LE
    gpio_clear(GPIOB, GPIO14); // Data
    gpio_clear(GPIOB, GPIO15); // Clk

    /* PB12 - ADF Chip Enable */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set(GPIOB, GPIO12);

    /* PA8 - ADF Lock Detect */
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8);

    usbd_device *const usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev,
        &config, usb_strings, 2, (uint8_t*)usbd_control_buffer,
        sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, usb_set_config_cb);

    while (1) {
        usbd_poll(usbd_dev);
    }
}

