package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDshooterTuning extends OpMode
{
    public DcMotorEx shooter;
    public double high = 1780; //P=7.6 F=15.2
    // 2000;P= 7.3 F = 15.1
    public double med = 1500; //P = 9.92 F = 15.19
    double targetVelocity = med;
    double P = 300.0;
    double F = 14.3;
    double[] step_sizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int step_index = 1;


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
    }

    @Override
    public void loop() {


        if (gamepad1.yWasPressed())
            if(targetVelocity==high)
            {
                targetVelocity = med;
            }
        else
            {
                targetVelocity = high;
            }

        if(gamepad1.bWasPressed())
        {
            step_index = (step_index + 1) % step_sizes.length;
        }

        if (gamepad1.dpadLeftWasPressed())
        {
            F -= step_sizes[step_index];
        }
        if (gamepad1.dpadRightWasPressed())
        {
            F += step_sizes[step_index];
        }

        if (gamepad1.dpadUpWasPressed())
        {
            P += step_sizes[step_index];
        }
        if (gamepad1.dpadDownWasPressed())
        {
            P -= step_sizes[step_index];
        }

        PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        shooter.setVelocity(targetVelocity);
        double curVelocity = shooter.getVelocity();
        double error = targetVelocity - curVelocity;

        telemetry.addData("Target Vleocity: ", targetVelocity);
        telemetry.addData("Current Velocity: ", curVelocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("Tuning P: ", "%4f (D-pad U/D)", P);
        telemetry.addData("Tuning F: ", "%4f (D-pad L/R)", F);
        telemetry.addData("Step Size: ", "%4f (B button)", step_sizes[step_index]);




    }
}
