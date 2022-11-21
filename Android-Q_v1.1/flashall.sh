OUTROOT=/home/Open-Q_865_Android-Q_v1.1/out/target/product/kona
adb reboot bootloader
fastboot --slot all flash boot $OUTROOT/boot.img
fastboot flash dtbo_a $OUTROOT/dtbo.img
fastboot flash dtbo_b $OUTROOT/dtbo.img
fastboot flash metadata $OUTROOT/metadata.img
fastboot flash persist $OUTROOT/persist.img
fastboot flash recovery_a $OUTROOT/recovery.img
fastboot flash recovery_b $OUTROOT/recovery.img
fastboot flash super $OUTROOT/super.img
fastboot flash vbmeta_a $OUTROOT/vbmeta.img
fastboot flash vbmeta_b $OUTROOT/vbmeta.img
fastboot flash vbmeta_system_a $OUTROOT/vbmeta_system.img
fastboot flash vbmeta_system_b $OUTROOT/vbmeta_system.img
fastboot erase userdata
fastboot reboot
