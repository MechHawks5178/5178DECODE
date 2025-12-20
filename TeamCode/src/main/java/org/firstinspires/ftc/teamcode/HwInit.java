package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class HwInit extends OpMode
{
    IMU imu;
    IMU.Parameters myIMUParameters;
    YawPitchRollAngles robotOrientation;
    DcMotorEx backRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx frontLeftMotor;
    DcMotor intake;
    DcMotor shooter;
    CRServo carousel;
    CRServo lift;

    Servo RGB_light;

    double speed;
    double speed_fine_inc = 0.05;
    boolean r_bump_1 = false;
    boolean l_bump_1 = false;
    boolean carousel_on = false;
    boolean move_to_shoot = false;
    boolean move_to_load = false;
    boolean lift_on = false;
    double carousel_speed = 0.22;
    MagneticLimit LoadSw = new MagneticLimit();
    MagneticLimit ShootSw = new MagneticLimit();
    ColorSensor color_sense = new ColorSensor();
    double robot_yaw;
    double robot_roll;
    double robot_pitch;

    int tolerance = 3;
    int posDriveWait= 1600;
    int posDriveStraightSize = 1000; // js about perfect
    int posDriveStrafeSize = 1075; // between 1060 and 1100
    int posDriveTurnSize = 960; // js about perfect

    public void Hw_init()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"backRightDrive");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeftDrive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRightDrive");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeftDrive");
        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        carousel = hardwareMap.crservo.get("carousel");
        lift = hardwareMap.crservo.get("lift");
        RGB_light = hardwareMap.get(Servo.class, "rgb_light");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LoadSw.init(hardwareMap, "load_switch");
        ShootSw.init(hardwareMap, "shoot_switch");
        color_sense.init(hardwareMap, "color_sensor");

        myIMUParameters= new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            new Orientation(
                                    AxesReference.INTRINSIC,
                                    AxesOrder.ZYX,
                                    AngleUnit.DEGREES,
                                    90f,
                                    0f,
                                    0f,
                                    0L
                            )
                    )
        );
        imu.initialize(myIMUParameters);

    }
    public void pos_drive_init()
    {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void posDrive(int position, int velocity, float rfDir, float lfDir, float rbDir, float lbDir) {

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setTargetPosition(Math.round(position * rfDir * 1));
        frontLeftMotor.setTargetPosition(Math.round(position * lfDir * -1));
        backRightMotor.setTargetPosition(Math.round(position * rbDir * 1));
        backLeftMotor.setTargetPosition(Math.round(position * lbDir * -1));

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setTargetPositionTolerance(tolerance);
        frontLeftMotor.setTargetPositionTolerance(tolerance);
        backRightMotor.setTargetPositionTolerance(tolerance);
        backLeftMotor.setTargetPositionTolerance(tolerance);

        frontRightMotor.setVelocity(velocity);
        frontLeftMotor.setVelocity(velocity);
        backRightMotor.setVelocity(velocity);
        backLeftMotor.setVelocity(velocity);
    }
    public void posStraight(float position, int velocity, int direction,float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveStraightSize),velocity,direction,direction,direction,direction);
        try {
            sleep(Math.round(position*wait*posDriveWait));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void posStrafe(float position, int velocity, int direction, float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveStrafeSize),velocity,-direction,direction,direction,-direction);
        try {
            sleep(Math.round(position*wait*posDriveWait));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void posTurn(float position, int velocity, int direction, float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveTurnSize),velocity,direction,-direction,direction,-direction);
        try {
            sleep(Math.round(position*wait*posDriveWait));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void shooter_off()
    {
        shooter.setPower(0.0);
    }
    public void shooter_on_far()
    {
        shooter.setPower(0.85);
    }
    public void shooter_on_mid()
    {
        shooter.setPower(0.75);
    }
    public void shooter_on_near()
    {
        shooter.setPower(0.70);
    }

    public void run_lift()
    {
        try {
            lift.setPower(1);
            sleep(2000);
            lift.setPower(-1);
            sleep(2200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        lift.setPower(0);
    }

    public void set_carousel_mode()
    {
       if(!move_to_load ||
          !move_to_shoot) {
           if (LoadSw.isLimitSwitchClosed()) {
               move_to_shoot = true;
           } else if (ShootSw.isLimitSwitchClosed()) {
               move_to_load = true;
           } else {
               move_to_load = true;
           }
       }
    }

    public void move_to_shoot_from_load(int dir)
    {
        carousel.setPower(dir * carousel_speed);
        if(ShootSw.isLimitSwitchClosed())
        {
            carousel.setPower(0.0);
            move_to_shoot = false;
        }
    }
    public void move_to_load_from_shoot(int dir)
    {
        carousel.setPower(dir * carousel_speed);
        if(LoadSw.isLimitSwitchClosed())
        {
            carousel.setPower(0.0);
            move_to_load = false;
        }
    }
    public void move_to_next_shoot_blocking()
    {
        carousel.setPower(carousel_speed);
        try{
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        do{
            carousel.setPower(carousel_speed);
        } while(!ShootSw.isLimitSwitchClosed());
        carousel.setPower(0.0);
    }

    public void update_light(ColorSensor.DetectedColor color)
    {
        switch (color) {
            case PURPLE:
                RGB_light.setPosition(0.700);
                break;
            case GREEN:
                RGB_light.setPosition(0.500);
                break;
            case UNK:
            default:
                RGB_light.setPosition(.275);

        }
    }
    public void update_imu()
    {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robot_yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        robot_roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        robot_pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
    }
}
