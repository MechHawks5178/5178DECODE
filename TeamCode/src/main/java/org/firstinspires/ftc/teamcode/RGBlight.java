package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

public class RGBlight {

    private Servo light;
    private final int blink_time = 500;
    private ElapsedTime blink_timer = new ElapsedTime(MILLISECONDS);
    private enum blink_state{OFF, DARK, LIT};
    private blink_state state;
    
    public void init(HardwareMap hwMap, String device_name)
    {
        light = hwMap.get(Servo.class, device_name);
	    state = blink_state.OFF;
    }

    public void light_on(double value)
    {
        light.setPosition(value);
    }
    public void light_off()
    {
        light.setPosition(0.0);
    }
    
    /*
      assumes it's being called repeatedly in an outside loop
     */
    public void blink(double value)
    {
	switch(state)
	{
	case OFF:
	    blink_timer.reset();
	    state = blink_state.LIT;
	    //intentional lack of break here
	    
	case LIT:
	    if (blink_timer.time() >= blink_time) {
		state =	blink_state.DARK;
		blink_timer.reset();
	    }
            else{
	        light.setPosition(value);
            }
	    //intentional lack of break here

	case DARK:
	    if (blink_timer.time() >= blink_time) {
		state = blink_state.LIT;
		blink_timer.reset();
	    }
	    else{
		light.setPosition(0.0);
	    }
	    break;
	}
    }
    public void stop_blink()
    {
	state = blink_state.OFF;
    }
}
