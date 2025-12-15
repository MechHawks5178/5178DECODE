package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MagneticLimit {
    private DigitalChannel limitMagnet;
    public void init(HardwareMap hwMap, String device_name){
        limitMagnet = hwMap.get(DigitalChannel.class, device_name);
        limitMagnet.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isLimitSwitchClosed(){
        return !limitMagnet.getState();
    }
}
