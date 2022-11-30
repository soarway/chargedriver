
#ifndef __SY_BC7D_HEADER__
#define __SY_BC7D_HEADER__

#define USB_NO_INPUT  0x0
#define USB_SDP       0x1
#define USB_CDP       0x2
#define USB_DCP       0x3
#define USB_HVDCP     0x4
#define USB_UNKNOWN   0x5
#define USB_NON_STD   0x6
#define USB_OTG       0x7

struct bc7d;

bool check_DCP_condition (struct bc7d *bq);

#endif


