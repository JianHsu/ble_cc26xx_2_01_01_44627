# invoke SourceDir generated makefile for appBLE.prm3
appBLE.prm3: .libraries,appBLE.prm3
.libraries,appBLE.prm3: package/cfg/appBLE_prm3.xdl
	$(MAKE) -f C:\ti\simplelink\ble_cc26xx_2_01_01_44627\Projects\ble\Free_Write\CC26XX\IAR\Config/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\simplelink\ble_cc26xx_2_01_01_44627\Projects\ble\Free_Write\CC26XX\IAR\Config/src/makefile.libs clean

