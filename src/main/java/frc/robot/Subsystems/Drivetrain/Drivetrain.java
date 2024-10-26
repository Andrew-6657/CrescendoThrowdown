package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.VisionFrame;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  // Drive motor controllers
  private final WPI_TalonSRX frontLeft;
  private final WPI_TalonSRX frontRight;
  private final WPI_TalonSRX backLeft;
  private final WPI_TalonSRX backRight;

  // Helper class for kinematics
  private final DifferentialDrive differentialDrive;

  // Gyro
  private final WPI_PigeonIMU gyro;

  // PID Controller for Speaker Alignment
  private PIDController alignPID = new PIDController(0, 0, 0); // TODO: Tune

  // Variables for tracking if we are aligned with the speaker
  private boolean allignmentInTolerance;
  private boolean vellocityInTolerance;

  public Drivetrain() {

    // Assign CAN IDs
    frontLeft = new WPI_TalonSRX(CANID.kFrontLeft);
    frontRight = new WPI_TalonSRX(CANID.kFrontRight);
    backLeft = new WPI_TalonSRX(CANID.kBackLeft);
    backRight = new WPI_TalonSRX(CANID.kBackRight);
    gyro = new WPI_PigeonIMU(CANID.kPigeon);

    // Reset configs to defaults
    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();

    // Configure Following
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // Enable Brake Mode
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

    // Configure Motor Inversion
    frontLeft.setInverted(InvertType.None);
    frontRight.setInverted(InvertType.InvertMotorOutput);

    backLeft.setInverted(InvertType.FollowMaster);
    backRight.setInverted(InvertType.FollowMaster);

    // Setup helper class
    differentialDrive = new DifferentialDrive(frontLeft, frontRight);

    // Apply current limits
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 40, 40, 0);
    frontLeft.configSupplyCurrentLimit(currentLimit);
    frontRight.configSupplyCurrentLimit(currentLimit);
    backLeft.configSupplyCurrentLimit(currentLimit);
    backRight.configSupplyCurrentLimit(currentLimit);
  }

  /**
   * Drives the robot with an Arcade Drive Style
   *
   * @param xSpeed Forward magnitude [-1, 1]
   * @param rSpeed Angular magnitude [-1, 1]
   * @param squareInputs
   */
  public void drive(double xSpeed, double rSpeed, boolean squareInputs) {
    differentialDrive.arcadeDrive(xSpeed, rSpeed, squareInputs);
  }

  /**
   * @return True if the drivetrain is aligned with the speaker, and no longer moving.
   */
  public boolean isAligned() {
    return allignmentInTolerance && vellocityInTolerance;
  }

  /**
   * Angles the robot towards the speaker if the center speaker tag is visible. (Will not move
   * otherwise)
   *
   * @param visionFrameSupplier Method that returns a VisionFrame object
   * @return Command to be ran
   */
  public Command speakerAlign(Supplier<VisionFrame> visionFrameSupplier) {
    return Commands.run(
        () -> {
          VisionFrame visionFrame = visionFrameSupplier.get();
          double output = visionFrame.hasTarget ? alignPID.calculate(visionFrame.tX, 0) : 0;

          allignmentInTolerance = MathUtil.isNear(0, visionFrame.tX, 120);
          vellocityInTolerance = MathUtil.isNear(0, gyro.getRate(), 5);

          drive(0, isAligned() ? 0 : output, false);
        });
  }

  /** Periodic logging of useful values. */
  @Override
  public void periodic() {
    Logger.recordOutput("Left Motors", frontLeft.get());
    Logger.recordOutput("Right Motors", frontRight.get());
    Logger.recordOutput("Left Current", frontLeft.getSupplyCurrent());
    Logger.recordOutput("Right Current", frontRight.getSupplyCurrent());
    Logger.recordOutput("Angular Velocity", gyro.getRate());
    Logger.recordOutput("Allignment in Tollerance", allignmentInTolerance);
    Logger.recordOutput("Velocity in Tollerance", vellocityInTolerance);
  }
}
