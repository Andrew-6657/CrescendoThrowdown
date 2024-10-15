package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

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
        public static double maxVoltage = 15.0;
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

        //kick constants (flywheal?)
    
      }
    public static class ShooterConstants{
      public static class RightFlywheel{
        public static final double kGearing = (1d / 2); //note, ask if this should be changed to a 2d / 1
        
        public static final double kMinRpm = -3190;
        public static final double kMaxRpm = 3190; //these need to be configured

        public static final double kCurrentLimit = 30;

        public static final double kFeedRPM = 300; //these need to be configured
        public static final double kSpeakerRPM = 2950;
        public static final double kAmpRPM = 1000;

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
      public static class LeftFlywheel{
        public static final double kGearing = (1d / 1); //I think that this one is a 1 to 1 ratio
        
        public static final double kMinRpm = -3190;
        public static final double kMaxRpm = 3190; //these need to be configured

        public static final double kCurrentLimit = 30;

        public static final double kFeedRPM = 300; //these need to be configured
        public static final double kSpeakerRPM = 2950;
        public static final double kAmpRPM = 1000;

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
