import SAMBA 3.2
import SAMBA.Connection.Serial 3.2
import SAMBA.Device.SAMV71 3.2
import "led.js" as Led

SerialConnection {

    device: SAMV71 {}

    onConnectionOpened: {
        Led.setup(this)

        // initialize internal flash applet
        initializeApplet("internalflash")
        
        //write files
        Led.blue_on(this)
        
        applet.erase(0, applet.memorySize)
        applet.write(0, "build/code.bin")
        applet.verify(0, "build/code.bin")

        // initialize boot config applet
		initializeApplet("bootconfig")
        
        // initialize boot config applet
        applet.writeBootCfg(BootCfg.BOOTMODE, BootCfg.BOOTMODE_FLASH)

        Led.off(this)

        for (var i = 0; i < 4; i++)
        {
            Utils.msleep(100)
            Led.on(this)
            Utils.msleep(100)
            Led.off(this)
        }
    }
}