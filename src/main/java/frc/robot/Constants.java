package frc.robot;

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
      POSITVIE(1),
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
        public static double maxVoltage = 15;
        public static class CAN{
          public static int kFrontLeft = 1;
          public static int kBackLeft = 2;
          public static int kFrontRight = 3;
          public static int kBackRight = 4;
          public static int kPigeon = 5;
          public static int kIntake = 6;
          public static int kPivot = 7;

        }
    }

    public static class IntakeConstants {
    
        //pivote contants

        //kick constants (flywheal?)
    
      }
}
