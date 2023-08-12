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
char ep0_buf[4][128] = {0};

int main(void) {
    asm("cpsid i");
    systick_init(32000, 1, 1, 1);
    status_leds_init();

    SCB->SHCSR |= (SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    // AXBS / CROSSBAR CONFIG
    // USBFS is Bus Master #4 on K64 (TRM 3.3.6.1)
    // FLASH controller is slave 0, SRAM backdoor is slave 1.
    // AIPSx_MPRA resets to 0x77700000, only ARM (ID)CODE, ARM SYS, and DMA are
    // "trusted" on boot. (TRM 3.3.8.3)
    // Why is it called backdoor?: TRM 3.5.3.3, M4 core gets its own direct access,
    // allowing for TCM to be built into the M4 even though it does not support that
    // officially from ARM
    //
    // AIPS / PERIPHERAL BRIDGE CONFIG
    // Flash memory is slot 32.
    // USBFS is slot 114.
    //
    // See TRM 3.3.6 for a graphical representation of the AXBS ports.
    // allow USBFS to read / write to anywhere in the address space. (no granularity)
    AIPS0->MPRA |= (AIPS_MPRA_MTR4_MASK | AIPS_MPRA_MTW4_MASK | AIPS_MPRA_MPL4_MASK);
    AIPS1->MPRA |= (AIPS_MPRA_MTR4_MASK | AIPS_MPRA_MTW4_MASK | AIPS_MPRA_MPL4_MASK);
    // ARM CODE and SYSTEM buses have lowest priority on SRAM backdoor
    // (should be using front door anyway)
    // can't write fields at once: two masters may not have the same prio at any time.
    AXBS->SLAVE[1].PRS |= AXBS_PRS_M0_MASK;
    AXBS->SLAVE[1].PRS &= ~AXBS_PRS_M1_MASK;
    AXBS->SLAVE[1].PRS |= (6 << 4); // priority 6 on M1 -> S1
    // USBFS has highest priority (0) on SRAM backdoor
    AXBS->SLAVE[1].PRS &= ~AXBS_PRS_M4_MASK;
    // allow USBFS to be interrupted after 1 "beat" in burst transfers.
    AXBS->MGPCR4 = 1;
    AXBS->SLAVE[1].CRS |= (1 << AXBS_CRS_ARB_SHIFT);

    SIM->SCGC4 &= ~SIM_SCGC4_USBOTG_MASK;

    // use USB 48MHz ref clock for PLL/FLL
    SIM->SOPT2 |= (SIM_SOPT2_PLLFLLSEL_MASK);
    // usb is fed from pll
    SIM->SOPT2 |= (SIM_SOPT2_USBSRC_MASK);

    // turn off MPU since apparently it conflicts with USB (mcuoneclipse dude)
    /*SIM->SCGC7 &= ~(SIM_SCGC7_MPU_MASK);*/
    SYSMPU->CESR &= ~1;

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
    /*USB0->USBTRC0 |= USB_USBTRC0_USBRESMEN_MASK;*/

    // tell usbfs where the bdt is
    USB0->BDTPAGE3 = ((uint32_t)&usb_endpoints[0]) >> 24;
    USB0->BDTPAGE2 = ((uint32_t)&usb_endpoints[0]) >> 16;
    USB0->BDTPAGE1 = (((uint32_t)&usb_endpoints[0]) >> 8) & 0xFE;

    // set up usb endpoint 0 tx
    usb_endpoints[2].buf_addr = (uint32_t)&self_descriptor;
    usb_endpoints[2].set.bd_ctrl = (1 << 1) | (1 << 5); // dts, own *inside* bd_ctrl
    usb_endpoints[2].set.bc = sizeof(struct libusb_device_descriptor);
    
    usb_endpoints[3].buf_addr = (uint32_t)&self_descriptor;
    usb_endpoints[3].bd_fields = 0; 
    usb_endpoints[3].set.bc = sizeof(struct libusb_device_descriptor);

    // set up usb endpoint 0 rx
    usb_endpoints[0].buf_addr = (uint32_t)ep0_buf[0];
    usb_endpoints[0].bd_fields = 0;
    usb_endpoints[0].set.bc = 64;
    // dts, own
    usb_endpoints[0].set.bd_ctrl |= ((1 << 1) | (1 << 5));

    usb_endpoints[1].buf_addr = (uint32_t)ep0_buf[1];
    usb_endpoints[1].bd_fields = 0;
    usb_endpoints[1].set.bc = 64;
    usb_endpoints[1].set.bd_ctrl |= ((1 << 1) | (1 << 5));

    USB0->ENDPOINT[0].ENDPT = 0x0D;
    for(int i = 1; i < 16; i++) {
        USB0->ENDPOINT[i].ENDPT = 0; // disable all other endpoints
    }

    // ask usbfs to interrupt us for a variety of conditions
    USB0->INTEN |= (USB_INTEN_TOKDNEEN_MASK | /* USB_INTEN_SOFTOKEN_MASK | */ USB_INTEN_USBRSTEN_MASK | USB_INTEN_ERROREN_MASK | USB_INTEN_STALLEN_MASK);
    USB0->OTGICR = 0; // disable otg interrupts

    USB0->ERREN = 0xFF;

    // disable suspend state
    USB0->USBCTRL &= ~(USB_USBCTRL_SUSP_MASK);

    // turn on the usbfs module
    USB0->CTL |= (USB_CTL_USBENSOFEN_MASK);

    NVIC_SetPriority(USB0_IRQn, 0);
    NVIC_ClearPendingIRQ(USB0_IRQn);
    NVIC_EnableIRQ(USB0_IRQn);
    asm("cpsie i");
    for(;;);
    return 0;
}

void clear_stall(uint8_t stat_reg) {
    usb_stat_t stat = (usb_stat_t)stat_reg;
    int ep_num = stat.fields.endp;
    // stat register is the address offset into the bdt of the affected ep
    uint8_t bdt_idx = stat_reg >> 3;

    // clear stall on the endpoint bit
    USB0->ENDPOINT[ep_num].ENDPT &= ~USB_ENDPT_EPSTALL_MASK;

    // clear stall in the bdt
    if(stat.fields.tx) {
        // tx dir stall
        usb_endpoints[bdt_idx].set.bd_ctrl = 0;
        // dts, own
        usb_endpoints[bdt_idx].set.bd_ctrl = ((1 << 1) | (1 << 5));
    }
    else {
        // rx dir stall
        usb_endpoints[bdt_idx].bd_fields = 0;
    }
}


bool usb_int_state = false;
volatile uint8_t istat;
volatile uint8_t errstat;
volatile usb_stat_t stat;
__attribute__((interrupt))
void USB0_IRQHandler(void) {
    istat = USB0->ISTAT;
    stat = (usb_stat_t)(USB0->STAT);
    errstat = USB0->ERRSTAT;
    USB0->OTGISTAT |= 0xff; // clear OTG flags which don't appear to have enables

    if(errstat) {
        asm("bkpt #0");
    }

    if(istat & USB_ISTAT_TOKDNE_MASK) {
        uint8_t bdt_idx = stat.raw >> 3;
        switch(usb_endpoints[bdt_idx].get.tok_pid) {
            case 0x09:
                // IN
                /*asm("bkpt #0");*/
            break;

            case 0x0D:
                // SETUP
                asm("bkpt #1");
            break;

            case 0x01:
                // OUT
                /*asm("bkpt #2");*/
            break;
        }
        RED_LED_ON();
    }

    if(istat & USB_ISTAT_USBRST_MASK) {
        USB0->ADDR = 0;
		USB0->CTL |= USB_CTL_ODDRST_MASK;
		USB0->CTL &= ~USB_CTL_ODDRST_MASK;
        USB0->ENDPOINT[0].ENDPT = 0x0D;
    }
    if(istat & USB_ISTAT_STALL_MASK) {
        // TODO handle un-stalling... why do the endpoints get stalled when
        // they are marked as OWN and TX?
        clear_stall(USB0->STAT);
    }

    // got setup packet
    if(USB0->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK) {
        // allow token processing to continue
        USB0->CTL &= ~(USB_CTL_TXSUSPENDTOKENBUSY_MASK);
        // dequeue other stuff here?
        asm("bkpt #5");
    }
    if(USB0->ERRSTAT & USB_ERRSTAT_DMAERR_MASK) {
        asm("bkpt #2");
    }

    // reset flags
    USB0->ISTAT = istat;
    USB0->ERRSTAT = errstat;
}

#define TICK_RELOAD 200
int tick_counter = TICK_RELOAD;
bool led_state = false;
__attribute__((interrupt))
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

__attribute__((interrupt))
void MemManage_Handler(void) {
    asm("bkpt #2");
}

__attribute__((interrupt))
void BusFault_Handler(void) {
    asm("bkpt #3");
}

__attribute__((interrupt))
void UsageFault_Handler(void) {
    asm("bkpt #4");
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
