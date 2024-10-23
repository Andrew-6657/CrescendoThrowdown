package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;

// import edu.wpi.first.math.util.Units;

public class Constants {

  public static class VisionFrame {
    public double tX = 0;
    public double tY = 0;
    public boolean hasTarget = false;

    public VisionFrame(double tX, double tY, boolean hasTarget) {
      this.tX = tX;
      this.tY = tY;
      this.hasTarget = hasTarget;
    }
  }

  public static class DriveConstants {
    public static double kTurboForwardSpeed = 1.0;
    public static double kNormalForwardSpeed = 0.4;
    public static double kTurboTurningSpeed = 0.7;
    public static double kNormalTurningSpeed = 0.5;

    // Note, enum is a special class for constants that uses comma list notation
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
      POSITVIE(1), // find out which direction is clockwise/counterclockwise
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

  public static class RobotConstants {
    public static double maxVoltage = 12.0;

    public static class CANID {
      public static int kFrontLeft = 1;
      public static int kBackLeft = 2;
      public static int kFrontRight = 3;
      public static int kBackRight = 4;
      public static int kIntake = 6;
      public static int kPivotL = 7;
      public static int kPivotR = 8;
      public static int kRightFlywheel = 9;
      public static int kLeftFlywheel = 10;
      public static int kIntakeTOF = 11;
    }
  }

  public static class PivotConstants {

    public static final double kPivotCurrentLimit = 30;

    public static final double posOffset = 0.1;

    public static final double minimumPosition = 0;
    public static final double maximumPosition = 104;

    public static final double setPointTolerance = 1.0; // degrees

    public static final double kGearing = 1d / (7 * 5 * 8);

    public static Slot0Configs kSlot0 = // These need to be tuned
        new Slot0Configs()
            .withKS(0)
            .withKV(12d / ((6380d / 60) * kGearing)) // Volts/Mechanism RPS
            .withKP(150)
            .withKI(0)
            .withKD(0);

    public static final CurrentLimitsConfigs kPivotCurrentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kPivotCurrentLimit)
            .withSupplyCurrentLimit(kPivotCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(kPivotCurrentLimit)
            .withSupplyTimeThreshold(0);
  }

  public static class ShooterConstants {

    public static class FlywheelSetPoint {

      public double leftRPM = 0;
      public double rightRPM = 0;

      public FlywheelSetPoint(double leftRPM, double rightRPM) {
        this.leftRPM =
            MathUtil.clamp(
                leftRPM, -6380d * LeftFlywheels.kGearing, 6380d * LeftFlywheels.kGearing);
        this.rightRPM =
            MathUtil.clamp(
                rightRPM, -6380d * RightFlywheels.kGearing, 6380d * RightFlywheels.kGearing);
      }
    }

    public static final FlywheelSetPoint kFeedRPM =
        new FlywheelSetPoint(300, 300); // these need to be configured
    public static final FlywheelSetPoint kSpeakerRPM = new FlywheelSetPoint(300, 300);
    public static final FlywheelSetPoint kAmpRPM = new FlywheelSetPoint(300, 300);
    public static final FlywheelSetPoint kIntake = new FlywheelSetPoint(-100, -100);

    public static class LeftFlywheels {
      public static final double kGearing = (1d / 1);

      public static final double kCurrentLimit = 40;

      public static final double setPointTolerance = 2.0; // rpm

      public static final CurrentLimitsConfigs kCurrentConfigs =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(kCurrentLimit * 2)
              .withSupplyCurrentLimit(kCurrentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentThreshold(kCurrentLimit)
              .withSupplyTimeThreshold(0);

      public static Slot0Configs kSlot0 =
          new Slot0Configs() // these values need to be tuned
              .withKS(.05)
              .withKV(12d / ((6380d / 60) * kGearing)) // Volts/Mechanism RPS
              .withKP(0)
              .withKI(0)
              .withKD(0);
    }

    public static class RightFlywheels {
      public static final double kGearing = (2d / 1);

      public static final double kCurrentLimit = 40;

      public static final double setPointTolerance = 2.0; // rmp

      public static final CurrentLimitsConfigs kCurrentConfigs =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(kCurrentLimit * 2)
              .withSupplyCurrentLimit(kCurrentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentThreshold(kCurrentLimit)
              .withSupplyTimeThreshold(0);

      public static Slot0Configs kSlot0 =
          new Slot0Configs() // these values need to be tuned
              .withKS(.05)
              .withKV(12d / ((6380d / 60) * kGearing)) // Volts/Mechanism RPS
              .withKP(0)
              .withKI(0)
              .withKD(0);
    }
  }
}
