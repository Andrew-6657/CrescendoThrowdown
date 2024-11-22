package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;

public class Constants {

  public static class RobotConstants {
    public static class CANID {

      // Drivetrain
      public static int kFrontLeft = 1;
      public static int kBackLeft = 2;
      public static int kFrontRight = 3;
      public static int kBackRight = 4;
      public static int kPigeon = 5;

      // Pivot
      public static int kPivotL = 6;
      public static int kPivotR = 7;

      // Shooter
      public static int kRightFlywheel = 8;
      public static int kLeftFlywheel = 9;
      public static int kChamberTOF = 10;
      public static int kKicker = 11;
    }
  }

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
    public static double kForwardSpeedMod = 0.4;
    public static double kTurningSpeedMod = 0.5;
  }

  public static class PivotConstants {

    public static final double kCurrentLimit = 30;

    public static final double minimumPosition = 0;
    public static final double maximumPosition = 104;

    public static final double positionTolerance = 5; // TODO: Tune

    public static final double encoderOffset = 0.774; // TODO: Tune
  }

  public static class ShooterConstants {

    // Object for storing a dual flywheel setpoint
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

    // Flywheel Setpoints
    public static final FlywheelSetPoint kSpeaker = new FlywheelSetPoint(2000, 4000);
    public static final FlywheelSetPoint kIntake = new FlywheelSetPoint(-3000, -3000);
    public static final FlywheelSetPoint kIdle = new FlywheelSetPoint(-750, -1500);
    public static final FlywheelSetPoint kStop = new FlywheelSetPoint(0, 0);
    public static final FlywheelSetPoint kSpit = new FlywheelSetPoint(600, 900);

    public static class LeftFlywheels {

      public static final double kGearing = (1d / 1);

      public static final double kCurrentLimit = 40;

      public static final double rpmTolerance = 2000.0; // TODO Tune

      public static final CurrentLimitsConfigs kCurrentConfigs =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(kCurrentLimit * 2)
              .withSupplyCurrentLimit(kCurrentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentThreshold(kCurrentLimit)
              .withSupplyTimeThreshold(0);

      // TODO: Tune
      public static Slot0Configs kSlot0 =
          new Slot0Configs()
              .withKS(.05)
              .withKV(12d / ((6380d / 60) * kGearing)) // Volts/Mechanism RPS
              .withKP(0)
              .withKI(0)
              .withKD(0);
    }

    public static class RightFlywheels {

      public static final double kGearing = (2d / 1);

      public static final double kCurrentLimit = 40;

      public static final double rpmTolerance = 2000.0; // rmp

      public static final CurrentLimitsConfigs kCurrentConfigs =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(kCurrentLimit * 2)
              .withSupplyCurrentLimit(kCurrentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentThreshold(kCurrentLimit)
              .withSupplyTimeThreshold(0);

      // TODO: Tune
      public static Slot0Configs kSlot0 =
          new Slot0Configs()
              .withKS(.05)
              .withKV(12d / ((6380d / 60) * kGearing)) // Volts/Mechanism RPS
              .withKP(0)
              .withKI(0)
              .withKD(0);
    }

    public static class Kicker {
      public static final double kCurrentLimit = 30;
    }
  }
}
