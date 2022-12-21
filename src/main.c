#include <systick.h>
#include <status_leds.h>

#include <MK64F12.h>

#include <string.h>
#include <stdbool.h>

/* blatant copypasta */
struct libusb_device_descriptor {
	/** Size of this descriptor (in bytes) */
	uint8_t  bLength;

	/** Descriptor type. Will have value
	 * \ref libusb_descriptor_type::LIBUSB_DT_DEVICE LIBUSB_DT_DEVICE in this
	 * context. */
	uint8_t  bDescriptorType;

	/** USB specification release number in binary-coded decimal. A value of
	 * 0x0200 indicates USB 2.0, 0x0110 indicates USB 1.1, etc. */
	uint16_t bcdUSB;

	/** USB-IF class code for the device. See \ref libusb_class_code. */
	uint8_t  bDeviceClass;

	/** USB-IF subclass code for the device, qualified by the bDeviceClass
	 * value */
	uint8_t  bDeviceSubClass;

	/** USB-IF protocol code for the device, qualified by the bDeviceClass and
	 * bDeviceSubClass values */
	uint8_t  bDeviceProtocol;

	/** Maximum packet size for endpoint 0 */
	uint8_t  bMaxPacketSize0;

	/** USB-IF vendor ID */
	uint16_t idVendor;

	/** USB-IF product ID */
	uint16_t idProduct;

	/** Device release number in binary-coded decimal */
	uint16_t bcdDevice;

	/** Index of string descriptor describing manufacturer */
	uint8_t  iManufacturer;

	/** Index of string descriptor describing product */
	uint8_t  iProduct;

	/** Index of string descriptor containing device serial number */
	uint8_t  iSerialNumber;

	/** Number of possible configurations */
	uint8_t  bNumConfigurations;
};

struct libusb_device_descriptor self_descriptor = {
    .bLength = sizeof(struct libusb_device_descriptor),
    .bDescriptorType = 0x01, // device
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x02, // communications class
    .bDeviceSubClass = 0x0, // defined to be zero, unused
    .bDeviceProtocol = 0x0, // unused
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1701,
    .idProduct = 0x6502,
    .iManufacturer = 0, // string 0 is manufacturer name
    .iProduct = 1, // string 1 is product name
    .iSerialNumber = 2, // string 2 is device serial number
    .bNumConfigurations = 1 // only one configuration for this device
};

typedef struct __attribute__((packed)) usbfs_bd_S {
	union {
		uint32_t bd_fields;

		struct {
			uint32_t reserved_1_0 : 2;
			uint32_t tok_pid : 4;
			uint32_t data01 : 1;
			uint32_t own : 1;
			uint32_t reserved_15_8 : 8;
			uint32_t bc : 16;
		} get __attribute__((packed));

		struct {
			uint32_t reserved_1_0 : 2;
			uint32_t bd_ctrl : 6;
			uint32_t reserved_15_8 : 8;
			uint32_t bc : 16;
		} set __attribute__((packed));

	} __attribute__((packed));
	uint32_t   buf_addr;
} usb_bd_t;

/*
 *typedef struct __attribute__((packed)) usbfs_bd_S {
 *    union {
 *        struct {
 *            uint8_t rsvd1:2;
 *            // buffer stall request
 *            uint8_t bdt_stall:1;
 *            // data toggle sync (data0/data1??)
 *            uint8_t dts:1;
 *            // no-increment (fifo mode, for isochronous endpoints)
 *            uint8_t ninc:1;
 *            // set this and own to 1 for isochronous (udp on schedule) endpoints.
 *            uint8_t keep:1;
 *            // data0 or data1 transferred to/from this endpoint next.
 *            uint8_t data01:1;
 *            // whether the buffer / bdt is owned by usbfs
 *            uint8_t own:1;
 *        } ctrl;
 *        struct {
 *            uint8_t rsvd2:2;
 *            uint8_t tok_pid:4;
 *            // unused for addressing the pid
 *            uint8_t rsvd3:2;
 *        } pid;
 *        uint8_t raw;
 *    };
 *    uint8_t rsvd4;
 *    // size of buffer (max 1024 per usb spec.)
 *    union {
 *        uint16_t byte_count:10;
 *        uint16_t bc:10;
 *    } __attribute__((packed));
 *    uint8_t rsvd5:6;
 *    uint32_t buffer_address;
 *} usb_bd_t;
 */

typedef union {
    struct __attribute__((packed)){
        uint8_t rsvd:2;
        uint8_t odd:1;
        uint8_t tx:1;
        uint8_t endp:4;
    } fields ;
    uint8_t raw;
} usb_stat_t;

// 4 total buffers for 1x endpoint: rxA, rxB, txA, txB
usb_bd_t usb_endpoints[4] __attribute__((aligned(512))) = {0};

// good thing we have plenty of ram :)
const char ep0_buf[4][128];

int main(void) {
    asm("cpsid i");
    systick_init(32000, 1, 1, 1);
    status_leds_init();

    // use USB 48MHz ref clock for PLL/FLL
    SIM->SOPT2 |= (SIM_SOPT2_PLLFLLSEL_MASK);
    // usb is fed from pll
    SIM->SOPT2 |= (SIM_SOPT2_USBSRC_MASK);

    // turn off MPU since apparently it conflicts with USB (mcuoneclipse dude)
    SIM->SCGC7 &= ~(SIM_SCGC7_MPU_MASK);

    SIM->CLKDIV2 &= ~(SIM_CLKDIV2_USBDIV_MASK);

    // enable USBFS peripheral
    SIM->SCGC4 |= SIM_SCGC4_USBOTG_MASK;

    // enable usb 48mhz ref. osc.
    USB0->CLK_RECOVER_IRC_EN |= USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK;
    // enable clock trimming via pll sync to host SYNC
    USB0->CLK_RECOVER_CTRL |= USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK;

    // enable D+ pullup (maybe frdm errata...? seems to make usb fs detect work)
    USB0->CONTROL |= (USB_CONTROL_DPPULLUPNONOTG_MASK);

    // set resume enable
    USB0->USBTRC0 |= USB_USBTRC0_USBRESMEN_MASK;

    // tell usbfs where the bdt is
    USB0->BDTPAGE3 = ((uint32_t)&usb_endpoints[0]) >> 24;
    USB0->BDTPAGE2 = ((uint32_t)&usb_endpoints[0]) >> 16;
    USB0->BDTPAGE1 = ((uint32_t)&usb_endpoints[0]) >> 8;


    // set up usb endpoint 0 tx
    usb_endpoints[2].buf_addr = (uint32_t)ep0_buf[0];
    usb_endpoints[2].set.bd_ctrl = (1) | (1 << 5); // dts, own
    
    usb_endpoints[3].buf_addr = (uint32_t)ep0_buf[1];
    usb_endpoints[3].set.bd_ctrl = 0;

    // set up usb endpoint 0 rx
    usb_endpoints[0].bd_fields = 0;
    usb_endpoints[0].buf_addr = (uint32_t)&self_descriptor;

    usb_endpoints[1].bd_fields = 0;
    usb_endpoints[1].buf_addr = (uint32_t)&self_descriptor;

    USB0->ENDPOINT[0].ENDPT = 0x0D;

    // ask usbfs to interrupt us for a variety of conditions
    USB0->INTEN |= (USB_INTEN_TOKDNEEN_MASK | /* USB_INTEN_SOFTOKEN_MASK | */ USB_INTEN_USBRSTEN_MASK | USB_INTEN_ERROREN_MASK);

    USB0->ERREN = 0xFF;

    // disable suspend state
    USB0->USBCTRL &= ~(USB_USBCTRL_SUSP_MASK);

    // turn on the usbfs module
    USB0->CTL |= (USB_CTL_USBENSOFEN_MASK);
    
    NVIC_SetPriority(USB0_IRQn, 0);
    NVIC_EnableIRQ(USB0_IRQn);
    asm("cpsie i");
    for(;;);
    return 0;
}


static bool usb_int_state = false;
void USB0_IRQHandler(void) {
    uint8_t istat = USB0->ISTAT;
    if(istat & USB_ISTAT_TOKDNE_MASK) {
        USB0->ISTAT |= (USB_ISTAT_TOKDNE_MASK);
        usb_stat_t stat = (usb_stat_t)(USB0->STAT);
        usb_bd_t *desc = usb_endpoints + stat.fields.endp;
        // IN token
        /*
         *if(desc->get.tok_pid == 0x9) {
         *    asm("bkpt #0");
         *}
         */
        // SETUP token
        /*
         *if(desc->get.tok_pid == 0xD) {
         *    asm("bkpt #1");
         *}
         */
        // OUT token
        /*
         *if(desc->get.tok_pid == 0x1) {
         *    asm("bkpt #1");
         *}
         */

        RED_LED_ON();
    }
    if(istat & USB_ISTAT_SOFTOK_MASK) {
        USB0->ISTAT |= (USB_ISTAT_SOFTOK_MASK);
        /*asm("bkpt #1");*/
    }
    if(istat & USB_ISTAT_USBRST_MASK) {
        USB0->ISTAT |= (USB_ISTAT_USBRST_MASK);
    }

    // got setup packet
    if(USB0->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK) {
        // allow token processing to continue
        USB0->CTL &= ~(USB_CTL_TXSUSPENDTOKENBUSY_MASK);
        // dequeue other stuff here?
    }
    if(USB0->ERRSTAT & USB_ERRSTAT_DMAERR_MASK) {
        /*asm("bkpt #1");*/
    }
    /*
     *if(usb_int_state) {
     *    RED_LED_ON();
     *}
     *else {
     *    RED_LED_OFF();
     *}
     *usb_int_state ^= 1;
     */
}

#define TICK_RELOAD 1000
static int tick_counter = TICK_RELOAD;
static bool led_state = false;
void SysTick_Handler(void) {
    if(tick_counter > 0) {
        tick_counter--;
    }
    else {
        tick_counter = TICK_RELOAD;
        if(led_state) {
            GREEN_LED_ON();
        }
        else {
            GREEN_LED_OFF();
        }
        led_state ^= 1;
    }
}
/*
        USB0->INTEN &= ~(USB_INTEN_RESUMEEN_MASK | USB_INTEN_SLEEPEN_MASK);
        USB0->CTL |= USB_CTL_ODDRST_MASK;
        USB0->ADDR = 0;

        USB0->ENDPOINT[0].ENDPT = USB_ENDPT_EPHSHK_MASK;
        for(int i = 1; i < 16; i++) {
            USB0->ENDPOINT[i].ENDPT = 0;
        }
        usb_endpoints[0].ctrl.data01 = 0;

        while(USB0->ENDPOINT[0].ENDPT & USB_ENDPT_EPTXEN_MASK);

        usb_endpoints[0].byte_count = 64;
        usb_endpoints[0].ctrl.data01 = 1;
        usb_endpoints[0].ctrl.dts = 1;
        usb_endpoints[0].ctrl.own = 1;

        usb_endpoints[1].raw = 0;
        usb_endpoints[1].ctrl.data01 = 0;

        USB0->ENDPOINT[0].ENDPT = 0x0D;
        USB0->INTEN |= (USB_INTEN_TOKDNEEN_MASK | USB_INTEN_SOFTOKEN_MASK | USB_INTEN_USBRSTEN_MASK | USB_INTEN_ERROREN_MASK);
*/
