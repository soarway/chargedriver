当前目录下面的Makefile的路径为：
./kernel/msm-4.19/Makefile

增加 -Wstrlcpy-strlcat-size \  ，避免strlcpy的警告被当成异常
KBUILD_AFLAGS   := -D__ASSEMBLY__
KBUILD_CFLAGS   := -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs \
		   -fno-strict-aliasing -fno-common -fshort-wchar \
		   -Werror-implicit-function-declaration \
		   -Wno-format-security \
		   -Wstrlcpy-strlcat-size \
		   -std=gnu89


./kernel/msm-4.19/include/linux/power_supply.h 
/kernel/msm-4.19/drivers/power/supply/power_supply_core.c
