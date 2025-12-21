package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mechawks-Tele-1")
public class   TeleOpMode extends HwInit {
//far and mid use the trigger buttons which are analog,
// but near is on the bumper which is just on/off
  float shooter_far_on = 0.0F;
  float shooter_mid_on = 0.0F;
  boolean shooter_near_on = false;
  boolean intake_on = false;
  boolean intake_clear = false;
  int carousel_dir = 1;




    public void init()
    {
        Hw_init();
        //set drive speed at 0.5 initially
        speed = 0.85;
        //initialise bumpers as "not pressed"
        r_bump_1 = false;
        l_bump_1 = false;
    }

    @Override
    public void loop() {

        ColorSensor.DetectedColor color = color_sense.getDetectedColor(telemetry);
        update_light(color);
        do_p1_things();
        do_p2_things();

        if(carousel_on)
        {
            set_carousel_mode();
        }
        if (move_to_load)
        {
            move_to_load_from_shoot(carousel_dir);
        }
        if (move_to_shoot)
        {
            move_to_shoot_from_load(carousel_dir);
        }

        run_motors();
        telemetry.update();
    }
    public void run_motors() {

        if (LoadSw.isLimitSwitchClosed()) {
            telemetry.addData("Load State", LoadSw.isLimitSwitchClosed());
        }else {
            telemetry.addData("Load State", LoadSw.isLimitSwitchClosed());
        }
        if (ShootSw.isLimitSwitchClosed()){
            telemetry.addData("Shoot State", ShootSw.isLimitSwitchClosed());
        }else {
            telemetry.addData("Shoot State", ShootSw.isLimitSwitchClosed());
        }

        if (lift_on) {
            if (ShootSw.isLimitSwitchClosed())
            {
                run_lift();
            }
        }else {
            lift.setPower(0);
        }

        if (intake_on)
        {
            intake.setPower(0.8);
        }
        else if (intake_clear)
        {
            intake.setPower(-0.4);
        }
        else
        {
            intake.setPower(0);
        }

        if (shooter_mid_on > 0.23)
        {
            shooter_on_mid();
        }else if (shooter_far_on  > 0.2)
        {
            shooter_on_far();
        }else if (shooter_near_on)
        {
            shooter_on_near();
        }else
        {
            shooter_off();
        }
    }

    public float avg(float[] nums) {
        int numlen = nums.length;
        float tot = 0;
        for (int i = 0; i < numlen; i++) {
            tot += nums[i];
        }
        tot /= numlen;
        return tot;
    }

    public void dual_joy_control(float left_stick_x, float left_stick_y, float right_stick_x, float right_stick_y) {
        /*TABLE OF INP
             LX  LY  RX
        RR   +   -   -
        RL   +   +   -
        FR   -   -   -
        FL   -   +   -
        */

        backRightMotor.setPower(speed * (left_stick_x - left_stick_y - right_stick_x));
        backLeftMotor.setPower(speed * (left_stick_x + left_stick_y - right_stick_x));
        frontRightMotor.setPower(speed * (-left_stick_x - left_stick_y - right_stick_x));
        frontLeftMotor.setPower(speed * (-left_stick_x + left_stick_y - right_stick_x));
    }

    public void p1_fine_speed_control() {
        if (gamepad1.a) {
            speed = 1;
        }
        if (gamepad1.x) {
            speed = 0.1;
        }
        if (gamepad1.y) {
            speed = 0.45;
        }
        if (gamepad1.b) {
            speed = 0.8;
        }
        if (gamepad1.right_bumper) {
            if (!r_bump_1) {
                speed += speed_fine_inc;
            }
            r_bump_1 = true;
        } else {
            r_bump_1 = false;
        }
        if (gamepad1.left_bumper) {
            if (!l_bump_1) {
                speed -= speed_fine_inc;
            }
            l_bump_1 = true;
        } else {
            l_bump_1 = false;
        }
    }
    public void do_p1_things() {
        p1_fine_speed_control();

        dual_joy_control(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y);
    }
    public void do_p2_things() {

        carousel_on = (gamepad2.x || gamepad2.b);

        if (gamepad2.xWasPressed())
        {
            carousel_dir = 1;
        }

        if (gamepad2.bWasPressed())
        {
             carousel_dir = -1;
        }

        intake_on = gamepad2.dpad_up;
        intake_clear = gamepad2.dpad_down;
        shooter_near_on = gamepad2.left_bumper;
        shooter_mid_on = gamepad2.left_trigger;
        shooter_far_on = gamepad2.right_trigger;
        lift_on = gamepad2.y;

    }
}