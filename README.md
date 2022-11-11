# driver

find  ./kernel/msm-4.19/drivers/ -name "Kconfig" -exec dos2unix {} \;

find  ./kernel/msm-4.19/drivers/ -name "Kconfig" -exec echo {} \; -exec grep -w  "USB_PD" {} \;

find ./kernel/msm-4.19/drivers/ -name "Kconfig" |xargs cat | xargs -w grep "USB_PD"
find ./kernel/msm-4.19/drivers/ -name "Kconfig" |-exec cat {} \;

find ./kernel/msm-4.19/drivers/usb/ -name "*.c" -exec echo {} \; -exec grep -w  "of_find_node_by_name" {} \;

find ./vendor/qcom/proprietary/devicetree-4.19/ -name "*.dtsi" -exec echo {} \; -exec grep -w  "displayport" {} \;

cp -r ./vendor/qcom/proprietary/devicetree-4.19/qcom/  /mnt/hgfs/Android-Q_v1.1/
cp -r ./kernel/msm-4.19/drivers/                       /mnt/hgfs/Android-Q_v1.1/
cp -r ./kernel/msm-4.19/include/                       /mnt/hgfs/Android-Q_v1.1/

设备树目录：${ROOT}/vendor/qcom/proprietary/devicetree-4.19/qcom/
源代码目录：${ROOT}/kernel/msm-4.19/drivers/
头文件目录：${ROOT}/kernel/msm-4.19/include/


cp -r   /mnt/hgfs/Android-Q_v1.1/qcom/         /home/Open-Q_865_Android-Q_v1.1/vendor/qcom/proprietary/devicetree-4.19/
cp -r   /mnt/hgfs/Android-Q_v1.1/drivers/      /home/Open-Q_865_Android-Q_v1.1/kernel/msm-4.19/
cp -r   /mnt/hgfs/Android-Q_v1.1/include/      /home/Open-Q_865_Android-Q_v1.1/kernel/msm-4.19/

./out/host/linux-x86/bin/dtc  ./vendor/qcom/proprietary/devicetree-4.19/qcom/kona.dtsi

头文件
RT1711H                     BA41
------------------------------------------------------------------
                            power_supply_old.h
						    wakelock.h
						    wakeup_reason.h
misc--------------------------------------------------------------
    rt-regmap.h             sy-regmap.h
	                        另外9个头文件
power-------------------------------------------------------------
    rt-charger.h            sy-charger.h
	
switch------------------------------------------------------------
    switch.h                switch.h
	
usb---------------------------------------------------------------
    rt1711.h                ba41.h
	rt1711h.h		        ba41a.h
	其他都能对应            其他都能对应
	
==================================================================
源文件
RT1711H                     BA41
------------------------------------------------------------------
misc 
    rt-regmap.c             sy-regmap.c
    Kconfig                 Kconfig
	Makefile                Makefile
	
power-------------------------------------------------------------
    rt-charger.c            无
	                        avs目录
							reset目录
							supply目录
							Kconfig
							Makefile

switch-------------------------------------------------------------
    switch_hd3ss460.c       未变
	switch_gpio.c           未变
	switch_class.c          未变
	Kconfig                 未变
	Makefile                未变


usb----------------------------------------------------------------
    目录pd                  目录pd
	     tcpc_rt1711.c            tcpc_ba41.c
	     tcpc_rt1711h.c           tcpc_ba41a.c
		                          policy_engine.c
							      qpnp-pdphy.c
								  usbpd.h
	--------------------------------------------------------------							  
	目录phy                 目录phy
	    class-dual-role.c         未变	 
		Kconfig                   未变
        Makefile                  未变		
		 
	Kconfig                       未变		
    Makefile	                  未变		
		 
		 
		 
		 
		 
		 
		 
		 
		 









