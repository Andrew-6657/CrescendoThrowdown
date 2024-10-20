package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class Constants {

  public static class DriveConstants{
    public static double kTurboForwardSpeed = 1.0;
    public static double kNormalForwardSpeed = 0.4;
    public static double kTurboTurningSpeed = 0.7;
    public static double kNormalTurningSpeed = 0.5;

    //Note, enum is a special class for constants that uses comma list notation
    public static enum FrontState {
      FORWARD(1),
      REVERSE(-1);

      public final double direction;

      /**
       * @param direction Motor Percentage
       */
      FrontState(double direction) {
        this.direction = direction;
      }

    }

    public static enum RotateState {
      POSITVIE(1), //find out which direction is clockwise/counterclockwise
      NEGATIVE(-1);

      public final double direction;

      /**
       * @param direction Motor Percentage
       */
      RotateState(double direction) {
        this.direction = direction;
      }

    }


  }

    public static class RobotConstants{
        public static double maxVoltage = 12.0;
        public static class CANID{
          public static int kFrontLeft = 1;
          public static int kBackLeft = 2;
          public static int kFrontRight = 3;
          public static int kBackRight = 4;
          public static int kPigeon = 5;
          public static int kIntake = 6;
          public static int kPivot = 7;
          public static int kRightFlywheel = 8;
          public static int kLeftFlywheel = 9;
        }
    }

    public static class PivoteConstants {
    
        //pivote contants

        //kick constants (flywheel?)
    
      }


    

    public static class ShooterConstants{

      public static class FlywheelSetPoint{ 
      double leftRPM;
      double rightRPM;

      public FlywheelSetPoint(double leftRPM, double rightRPM){
        this.leftRPM = MathUtil.clamp(leftRPM, -6380d * LeftFlywheels.kGearing, 6380d * LeftFlywheels.kGearing);
        this.rightRPM = MathUtil.clamp(rightRPM, -6380d * RightFlywheels.kGearing, 6380d * RightFlywheels.kGearing);
      }

      // public void ChangeSetPoint(double leftRPM, double rightRPM){
      //   this.leftRPM = MathUtil.clamp(leftRPM, -6380d * LeftFlywheels.kGearing, 6380d * LeftFlywheels.kGearing);
      //   this.rightRPM = MathUtil.clamp(rightRPM, -6380d * RightFlywheels.kGearing, 6380d * RightFlywheels.kGearing);
      // }
    }

    

    public static final FlywheelSetPoint kFeedRPM = new FlywheelSetPoint(300, 300); //these need to be configured
    public static final FlywheelSetPoint kSpeakerRPM = new FlywheelSetPoint(300, 300);
    public static final FlywheelSetPoint kAmpRPM = new FlywheelSetPoint(300, 300);
    public static final FlywheelSetPoint kIntake = new FlywheelSetPoint(-100, -100);


      public static class RightFlywheels{
        public static final double kGearing = (2d / 1);
        

        public static final double kCurrentLimit = 40;


        public static final CurrentLimitsConfigs kCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kCurrentLimit * 2)
            .withSupplyCurrentLimit(kCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kCurrentLimit)
            .withSupplyTimeThreshold(0);

        public static Slot0Configs kSlot0 =
        new Slot0Configs() //these values need to be tuned
            .withKS(.05)
            .withKV(12d / ((6380d / 60) * kGearing)) // Volts/Mechanism RPS
            .withKP(0)
            .withKI(0)
            .withKD(0);
      }
      public static class LeftFlywheels{
        public static final double kGearing = (1d / 1); //I think that this one is a 1 to 1 ratio

        public static final double kCurrentLimit = 40;


        public static final CurrentLimitsConfigs kCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kCurrentLimit * 2)
            .withSupplyCurrentLimit(kCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kCurrentLimit)
            .withSupplyTimeThreshold(0);

        public static Slot0Configs kSlot0 =
        new Slot0Configs() //these values need to be tuned
            .withKS(.05)
            .withKV(12d / ((6380d / 60) * kGearing)) // Volts/Mechanism RPS
            .withKP(0)
            .withKI(0)
            .withKD(0);
      }
    }
    
}
