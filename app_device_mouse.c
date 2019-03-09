/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
 *******************************************************************************/

/** INCLUDES *******************************************************/
#include "system.h"

#include <stdint.h>

#include "usb.h"
#include "usb_device.h"
#include "usb_device_hid.h"

#include "app_led_usb_status.h"
#include "app_device_mouse.h"
#include "usb_config.h"

#include "LCD.h"

#if(__XC8)
#define PACKED
#else
#define PACKED __attribute__((packed))
#endif

/*******************************************************************************
 * HID Report Descriptor - this describes the data format of the reports that
 * are sent between the host and the device.
 *
 * In this example there are only one INPUT report.  This report descriptor can
 * be generated using the HID descriptor tool available at www.usb.org.
 ******************************************************************************/
const struct {
    uint8_t report[HID_RPT01_SIZE];
} hid_rpt01 ={
    {
        0x05, 0x01, /* Usage Page (Generic Desktop)             */
        0x09, 0x02, /* Usage (Mouse)                            */
        0xA1, 0x01, /* Collection (Application)                 */
        0x09, 0x01, /*  Usage (Pointer)                         */
        0xA1, 0x00, /*  Collection (Physical)                   */
        0x05, 0x09, /*      Usage Page (Buttons)                */
        0x19, 0x01, /*      Usage Minimum (01)                  */
        0x29, 0x03, /*      Usage Maximum (03)                  */
        0x15, 0x00, /*      Logical Minimum (0)                 */
        0x25, 0x01, /*      Logical Maximum (1)                 */
        0x95, 0x03, /*      Report Count (3)                    */
        0x75, 0x01, /*      Report Size (1)                     */
        0x81, 0x02, /*      Input (Data, Variable, Absolute)    */
        0x95, 0x01, /*      Report Count (1)                    */
        0x75, 0x05, /*      Report Size (5)                     */
        0x81, 0x01, /*      Input (Constant)    ;5 bit padding  */
        0x05, 0x01, /*      Usage Page (Generic Desktop)        */
        0x09, 0x30, /*      Usage (X)                           */
        0x09, 0x31, /*      Usage (Y)                           */
        0x15, 0x81, /*      Logical Minimum (-127)              */
        0x25, 0x7F, /*      Logical Maximum (127)               */
        0x75, 0x08, /*      Report Size (8)                     */
        0x95, 0x02, /*      Report Count (2)                    */
        0x81, 0x06, /*      Input (Data, Variable, Relative)    */
        0xC0, 0xC0 /* End Collection,End Collection            */
    }
};

/*******************************************************************************
 * Report Data Types - These typedefs will mirror the reports defined in the
 * HID report descriptor so that the application has an easy way to see what
 * the report will look like as well as access/modify each of the members of the
 * report.
 ******************************************************************************/

/* INPUT report - this structure will represent the only INPUT report in the HID
 * descriptor.
 */
typedef struct PACKED {

    /* The first INPUT item is the following:
     *   0x05, 0x09,    //Usage Page (Buttons)
     *   0x19, 0x01,    //Usage Minimum (01)
     *   0x29, 0x03,    //Usage Maximum (03)
     *   0x15, 0x00,    //Logical Minimum (0)
     *   0x25, 0x01,    //Logical Maximum (1)
     *   0x95, 0x03,    //Report Count (3)
     *   0x75, 0x01,    //Report Size (1)
     *   0x81, 0x02,    //Input (Data, Variable, Absolute)
     *
     * The usage page is buttons
     * The report size is 1 (1-bit)
     * The report count is 3, thus 3 1-bit items
     * The Usage Min is 1 and the Usage maximum is 3, thus buttons 1-3, also
     *   call the primary, secondary, and tertiary buttons.
     *
     * The second INPUT item comes from the fact that the report must be byte
     * aligned, so we need to pad the previous 3-bit report with 5-bits of
     * constant(filler) data.
     *   0x95, 0x01,    //Report Count (1)
     *   0x75, 0x05,    //Report Size (5)
     *   0x81, 0x01,    //Input (Constant)
     */
    union PACKED {

        struct PACKED {
            unsigned button1 : 1;
            unsigned button2 : 1;
            unsigned button3 : 1;
            unsigned : 5;
        };

        struct PACKED {
            unsigned primary : 1;
            unsigned secondary : 1;
            unsigned tertiary : 1;
            unsigned : 5;
        };
        uint8_t value;
    } buttons;

    /* The final INPUT item is the following:
     *   0x05, 0x01,    //Usage Page (Generic Desktop)
     *   0x09, 0x30,    //Usage (X)
     *   0x09, 0x31,    //Usage (Y)
     *   0x15, 0x81,    //Logical Minimum (-127)
     *   0x25, 0x7F,    //Logical Maximum (127)
     *   0x75, 0x08,    //Report Size (8)
     *   0x95, 0x02,    //Report Count (2)
     *   0x81, 0x06,    //Input (Data, Variable, Relative)
     *
     * The report size is 8 (8-bit)
     * The report count is 2, thus 2 bytes of data.
     * The first usage is (X) and the second is (Y) so the first byte will
     *   represent the X mouse value, and the second the Y value.
     * The logical min/max determines the bounds for X and Y, -127 to 127.
     * The INPUT type is relative so each report item is relative to the last
     *   report item.  So reporting "-1" for X means that since the last report
     *   was sent, the mouse has move left
     */
    uint8_t x;
    uint8_t y;
} MOUSE_REPORT;

/** VARIABLES ******************************************************/
/* Some processors have a limited range of RAM addresses where the USB module
 * is able to access.  The following section is for those devices.  This section
 * assigns the buffers that need to be used by the USB module into those
 * specific areas.
 */
#if defined(FIXED_ADDRESS_MEMORY)
#if defined(COMPILER_MPLAB_C18)
#pragma udata MOUSE_REPORT_DATA_BUFFER=MOUSE_REPORT_DATA_BUFFER_ADDRESS
static MOUSE_REPORT mouseReport;
#pragma udata
#elif defined(__XC8)
static MOUSE_REPORT mouseReport MOUSE_REPORT_DATA_BUFFER_ADDRESS;
#endif
#else
static MOUSE_REPORT mouseReport;
#endif

typedef struct {

    struct {
        USB_HANDLE handle;
        uint8_t idleRate;
        uint8_t idleRateSofCount;
    } inputReport[1];

} MOUSE;

static MOUSE mouse;

int ReadPot(void);

/*********************************************************************
 * Function: void APP_DeviceMouseInitialize(void);
 *
 * Overview: Initializes the demo code
 *
 * PreCondition: None
 *
 * Input: None
 *
 * Output: None
 *
 ********************************************************************/
void APP_DeviceMouseInitialize(void) {
    /* Enable emulation mode.  This means that the mouse data
     * will be send to the PC causing the mouse to move.  If this is
     * set to false then the demo board will send 0,0,0 resulting
     * in no mouse movement.
     */
    LCDInit();
    lprintf(0, "USB Mouse Demo");
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB2 = 1;
    ANCON0 = 0x01;
    ANCON1 = 0x1f;
    ADCON0=0x01;				\
    ADCON1=0x9E;
    TRISAbits.TRISA0 = 1;
    
    /* initialize the handles to invalid so we know they aren't being used. */
    mouse.inputReport[0].handle = NULL;

    //enable the HID endpoint
    USBEnableEndpoint(HID_EP, USB_IN_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
}//end UserInit

/*********************************************************************
 * Function: void APP_DeviceMouseTasks(void);
 *
 * Overview: Keeps the demo running.
 *
 * PreCondition: The demo should have been initialized and started via
 *   the APP_DeviceMouseInitialize() and APP_DeviceMouseStart() demos
 *   respectively.
 *
 * Input: None
 *
 * Output: None
 *
 ********************************************************************/
void APP_DeviceMouseTasks(void) {

    /* If the USB device isn't configured yet, we can't really do anything
     * else since we don't have a host to talk to.  So jump back to the
     * top of the while loop. */
    if (USBGetDeviceState() < CONFIGURED_STATE) {
        return;
    }

    /* If we are currently suspended, then we need to see if we need to
     * issue a remote wakeup.  In either case, we shouldn't process any
     * keyboard commands since we aren't currently communicating to the host
     * thus just continue back to the start of the while loop. */
    if (USBIsDeviceSuspended() == true) {
        return;
    }
    int adc = ReadPot();
    if (HIDTxHandleBusy(mouse.inputReport[0].handle) == false) {
        mouseReport.buttons.value = 0;
        mouseReport.x = (adc - 512) / 128;
        mouseReport.y = 0;
        mouse.inputReport[0].handle = HIDTxPacket(
                HID_EP,
                (uint8_t*) & mouseReport,
                sizeof (mouseReport)
                );
    }

    
}//end ProcessIO

void APP_DeviceMouseIdleRateCallback(uint8_t reportId, uint8_t idleRate) {
    //Make sure the host is requesting to set the idleRate on a legal/implemented
    //report ID.  In applications that don't implement report IDs (such as this
    //firmware) the value should be == 0.
    if (reportId == 0) {
        mouse.inputReport[reportId].idleRate = idleRate;
    }
}

/*******************************************************************************
 * Function: void APP_DeviceMouseSOFHandler(void)
 *
 * Overview: Handles SOF events.  This is used to calculate the mouse movement
 *           based on the SOF counter instead of a device timer or CPU clocks.
 *           It can also be used to handle idle rate issues, if applicable for
 *           the demo.
 *
 * Input: none
 * Output: none
 *
 ******************************************************************************/
void APP_DeviceMouseSOFHandler(void) {
    /* We will be getting SOF packets before we get the SET_CONFIGURATION
     * packet that will configure this device, thus, we need to make sure that
     * we are actually initialized and open before we do anything else,
     * otherwise we should exit the function without doing anything.
     */
    if (USBGetDeviceState() != CONFIGURED_STATE) {
        return;
    }
}

int ReadPot(void) {
    ADCON0bits.CHS = 0;
    GODONE = 1;
    while (GODONE);
    return ADRES;
}
