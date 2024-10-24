package frc.robot.Subsystems.Drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.FrontState;
import frc.robot.Constants.DriveConstants.RotateState;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.VisionFrame;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX mFrontLeft;
  private final WPI_TalonSRX mFrontRight;
  private final WPI_TalonSRX mBackLeft;
  private final WPI_TalonSRX mBackRight;

  private final DifferentialDrive mDifferentialDrive;
  private FrontState mCurrentState;
  private RotateState mCurrentRotateState;

  private final WPI_PigeonIMU mPigeon;




  public Drivetrain() {
    mPigeon = new WPI_PigeonIMU(CANID.kPigeon);

    mFrontLeft = new WPI_TalonSRX(CANID.kFrontLeft);
    mFrontRight = new WPI_TalonSRX(CANID.kFrontRight);
    mBackLeft = new WPI_TalonSRX(CANID.kBackLeft);
    mBackRight = new WPI_TalonSRX(CANID.kBackRight);

    mFrontLeft.configFactoryDefault();
    mFrontRight.configFactoryDefault();
    mBackLeft.configFactoryDefault();
    mBackRight.configFactoryDefault();

    mBackLeft.follow(mFrontLeft);
    mBackRight.follow(mFrontRight);

    mFrontLeft.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mFrontRight.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mBackLeft.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mBackRight.configVoltageCompSaturation(RobotConstants.maxVoltage);

    mFrontLeft.enableVoltageCompensation(true);
    mFrontRight.enableVoltageCompensation(true);
    mBackLeft.enableVoltageCompensation(true);
    mBackRight.enableVoltageCompensation(true);

    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);

    mFrontLeft.setInverted(InvertType.None);
    mBackLeft.setInverted(InvertType.None); // REVIEW COMMENT: This should follow master
    mBackLeft.follow(mFrontLeft);

    mFrontRight.setInverted(InvertType.InvertMotorOutput);
    mBackRight.setInverted(InvertType.InvertMotorOutput); // REVIEW COMMENT: This should follow master
    mBackRight.follow(mFrontRight);

    mDifferentialDrive = new DifferentialDrive(mFrontLeft, mFrontRight);

    // REVIEW COMMENT: Reminder to look at current limmit again with a mentor
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 40, 40, 0);
    mFrontLeft.configSupplyCurrentLimit(currentLimit);
    mFrontRight.configSupplyCurrentLimit(currentLimit);
    mBackLeft.configSupplyCurrentLimit(currentLimit);
    mBackRight.configSupplyCurrentLimit(currentLimit);
  }

  public Rotation2d getAngle(){
    return mPigeon.getRotation2d().times(-1);
  }

  public double getPitch(){
    return mPigeon.getPitch();
  }

  public void drive(double xSpeed, double rSpeed) {
    mDifferentialDrive.arcadeDrive(
        xSpeed * mCurrentState.direction, -rSpeed * mCurrentState.direction, true);
  }


  public Command changeState(FrontState frontState) {
    return new InstantCommand(() -> mCurrentState = frontState);
  }

  public Command changeRotateState(RotateState RotateState) {
    return new InstantCommand(() -> mCurrentRotateState = RotateState);
  }

  private PIDController allignPID = new PIDController(0, 0, 0);

  

  public boolean drivetrainAlligned(Supplier<VisionFrame> visionFrameSupplier){
    VisionFrame visionFrame = visionFrameSupplier.get();
    boolean allignmentInTolerance = MathUtil.isNear(0, visionFrame.tX, 120); //tolerance needs tuning
    boolean vellocityInTolerance = MathUtil.isNear(0, mPigeon.getRate(), 5); //tollercance needs tuning
    return allignmentInTolerance && vellocityInTolerance;
  }

  public Command allignDrivetrainFromVision(Supplier<VisionFrame> visionFrameSupplier) {
    return Commands.run(
        () -> {
          VisionFrame visionFrame = visionFrameSupplier.get();
          double output = allignPID.calculate(visionFrame.tX,0);

          if(drivetrainAlligned(visionFrameSupplier)){
            drive(0, 0);
          } else {
            drive(0, output);
          }
        });
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Front Left Motor", mFrontLeft.get());
    Logger.recordOutput("Back Left Motor", mBackLeft.get());
    Logger.recordOutput("Front Right Motor", mFrontRight.get());
    Logger.recordOutput("Back Right Motor", mBackRight.get());
    Logger.recordOutput("Foward", mCurrentState.direction);
    Logger.recordOutput("Turn", mCurrentRotateState.direction);
  }
}
