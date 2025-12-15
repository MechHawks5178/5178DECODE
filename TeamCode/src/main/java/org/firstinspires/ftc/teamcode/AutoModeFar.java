package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto Mode Far Blue")
public class AutoModeFarBlue extends HwInit {

    boolean DEBUG_ON = false;

    public void init()
    {
        Hw_init();
        pos_drive_init();
    }

  @Override
  public void start()
  {
      // Assumptions
      // start at wall farthest from goal
      // start with robot aimed correctly at goal
      // start with part of robot touching back wall
      // start within or touching shooting line (triangle)
      // start with artifacts loaded in correct motif with first ball to shoot in position

      // actions
      // turn shooter motor on to far speed

      shooter_on_far();
      try{
          //TODO: adjust this time if needed
          sleep(1000);
      }
      catch (InterruptedException e)
      {
          throw new RuntimeException(e);
      }
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
      //TODO: might need a short sleep here to hold the last shoot power
      shooter_off();

      // drive forward out of shoot zone
      posStraight(1,1500, 1, 1);

  }

  @Override
    public void loop() {

      update_imu();

      if (DEBUG_ON)
      {
          if (LoadSw.isLimitSwitchClosed()) {
              telemetry.addData("Load State", LoadSw.isLimitSwitchClosed());
          } else {
              telemetry.addData("Load State", LoadSw.isLimitSwitchClosed());
          }
          if (ShootSw.isLimitSwitchClosed()) {
              telemetry.addData("Shoot State", ShootSw.isLimitSwitchClosed());
          } else {
              telemetry.addData("Shoot State", ShootSw.isLimitSwitchClosed());
          }
      }
  }

}
