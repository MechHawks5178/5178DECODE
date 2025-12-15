package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto Mode Near Blue")
public class AutoModeNearBlue extends HwInit {

    public void init()
    {
        Hw_init();
    }
    @Override
    public void start()
    {

        // start touching Blue goal
        // start with robot aimed correctly at goal
        // start with part of robot touching  goal
        // start within or touching shooting line (triangle)
        // start with artifacts loaded in correct motif with first ball to shoot in position

        // actions
        // turn shooter motor on to far speed
        shooter_on_near();
        //go to good shoot place
        posStraight(3.5f,1500, -1, 0);

        // raise shooter lift
        run_lift();
        // turn carousel to next shoot position
        move_to_next_shoot_blocking();
        // raise shooter lift
        run_lift();
        // turn carousel to next shoot position
        move_to_next_shoot_blocking();
        // raise shooter lift
        run_lift();
        // drive out of shoot zone
        posStrafe(1,1500, -1,0);
        //TODO: probably backwards. think about how you can do this the same on
        //  both sides so you can ignore red/blue. what if other robot is in your spot?







    }


    @Override
    public void loop() {

        update_imu();

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
        /*
        if (lift_on) {

            try {
                lift.setPower(1);
                sleep(2000);
                lift.setPower(-1);
                sleep(2000);
            } catch (InterruptedException e) { //WHY IS IT DOING THIS PLEASE HELP
                throw new RuntimeException(e);
            }
            lift.setPower(0);
        }else {
            lift.setPower(0);
        }*/

        //forward 5 secs (go to shooting zone (probably))
/*timer.reset();
        while (timer.time() <=5){
            frontRightMotor.setPower(1);
            frontLeftMotor.setPower(1);
            backRightMotor.setPower(1);
            backLeftMotor.setPower(1);
        }*/
        //total time:5

        //launch shooter
        /*timer.reset();
        while (timer.time() <=5){
            //lift on
            //shooter on
        }*/

        //launch shooter
       /* timer.reset();
        while (timer.time() <=5){
            //lift on
            //shooter on
        }*/

        //launch shooter
        /*timer.reset();
        while (timer.time() <=6){
            //lift on
            //shooter on
        }*/
        //time:30ish
    }
}
