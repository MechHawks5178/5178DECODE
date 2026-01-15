package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto Mode Near Blue")
public class AutoModeNearBlue extends HwInit
{
    @Override
    public void init() {
        Hw_init();
        update_light("BLUE");
    }

    @Override
    public void start()
    {
        // Assumptions
        // start at wall closest to pattern tag
        // start with part of robot touching front wall
        // start within or touching shooting line (triangle)
        // start with artifacts loaded with first ball in shoot position below

        //      Intake
        // Green --- Purple
        //     \\    //
        //      Purple
        //      Shooter


        // actions

        //assumes above load positions and normal Clockwise operation
        char[] load_pattern = {'P','P','G'};
        int carousel_pos = 0;

        // actions

        // drive backwards within shoot zone
        posStraight(2.5F,1500, -1, 1);
        //rotate 45 CCW
        posTurn(0.5F,1500, -1, 1);

        LimeLightRead();
        telemetry.addData("current tag: ", current_tag);
        char[] pattern = tag_to_pattern(current_tag);
        //TODO: check for null on pattern. this means it did not find tag 21 - 23 and doesn't know what pattern
        // easy response is just fire what you've got, advanced response is look for it
        if (null == pattern)
        {
            pattern = load_pattern;
        }

        // turn shooter motor on to far speed
        shooter_on_mid();

        //rotate 45 CCW
        posTurn(0.5F,1500, 1, 1);

        //TODO Verify Tag

        //shoot
        try{
            //TODO: adjust this time if needed
            sleep(1300);
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException(e);
        }

        for (int i=0; i < pattern.length; i++)
        {
            update_light( pattern[i]=='P'? "PURPLE" : "GREEN" );
            telemetry.addData("pattern color: ", pattern[i]);
            telemetry.addData("loop: ", i);
            telemetry.addData("carousel_pos; ", carousel_pos);

            if (pattern[i] == load_pattern[carousel_pos])
            {
                //First one correct, fire in the hole
                run_lift_blocking();
                telemetry.addLine("FIRE 1!");
            }
            else
            {
                if (pattern[i] == load_pattern[(carousel_pos + 1) % 3]) {
                    move_to_next_shoot_blocking(1);
                    carousel_pos = (carousel_pos + 1) % 3;
                    //FIRE!
                    run_lift_blocking();
                    telemetry.addLine("FIRE 2!");
                }
                else if (pattern[i] == load_pattern[(carousel_pos + 2) % 3]) {
                    move_to_next_shoot_blocking(-1);
                    carousel_pos = (carousel_pos + 2) % 3;
                    //FIRE!
                    run_lift_blocking();
                    telemetry.addLine("FIRE 3!");
                }
            }
            load_pattern[carousel_pos] = 'E';
            //update_light("UNK");
        }


        // drive out of shoot zone
        posStrafe(1,1500, -1,1);
        shooter_off();
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
        telemetry.addData("Shoot Pos Switch: ", shooterPosSw.isLimitSwitchPressed());

    }



}


