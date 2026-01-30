package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto Mode Far Comp1")
public class AutoModeFarComp1 extends HwInit {

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
      LimeLightRead();
      telemetry.addData("current tag: ", current_tag);
      char[] pattern = tag_to_pattern(current_tag);
      //TODO: check for null on pattern. this means it did not find tag 21 - 23 and doesn't know what pattern
      // easy response is just fire what you've got, advanced response is look for it
      if (null == pattern)
      {
          pattern = load_pattern;
      }
      final char[] blink = pattern;
      // FTC: dont use threads
      // me: lolwut
      /*new Thread() {
          @Override public void run() { blink_pattern(blink); }*/


      // turn shooter motor on to far speed
      shooter_on_far();
      try{
          lift.setPower(-1);
          sleep(200);
          lift.setPower(0);
          //TODO: adjust this time if needed
          sleep(1400);
      }
      catch (InterruptedException e)
      {
          throw new RuntimeException(e);
      }

      for (int i=0; i < pattern.length; i++)
      {
          //update_light( pattern[i]=='P'? "PURPLE" : "GREEN" );
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

    //
    // only blinks purple and green. not suitable for generic use
    // Do not use outside of here unless you understand the threading implications.
    // Here Be Dragons
    synchronized public void blink_pattern(char[] pattern)
    {
        for (char c : pattern) {
            try {
                update_light(c == 'P' ? "PURPLE" : "GREEN");
                sleep(1000);
                update_light("UNK");
                sleep(700);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

    }

}
