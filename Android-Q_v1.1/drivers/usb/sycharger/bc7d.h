
#ifndef __SY_BC7D_HEADER__
#define __SY_BC7D_HEADER__



enum bc7d_vbus_type {
	BC7D_VBUS_NONE,
	BC7D_VBUS_USB_SDP,
	BC7D_VBUS_USB_CDP, /*CDP for bq25890, Adapter for bq25892*/
	BC7D_VBUS_USB_DCP,
	BC7D_VBUS_MAXC,
	BC7D_VBUS_UNKNOWN,
	BC7D_VBUS_NONSTAND,
	BC7D_VBUS_OTG,
	BC7D_VBUS_TYPE_NUM,
};


struct bc7d_chip;

bool check_DCP_condition (struct bc7d_chip *bq);

#endif


