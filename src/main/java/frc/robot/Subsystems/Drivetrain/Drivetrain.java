package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.FrontState;
import frc.robot.Constants.DriveConstants.RotateState;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.CANID;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX mFrontLeft;
  private final WPI_TalonSRX mFrontRight;
  private final WPI_TalonSRX mBackLeft;
  private final WPI_TalonSRX mBackRight;

  private final DifferentialDrive mDifferentialDrive;
  private FrontState mCurrentState;
  private RotateState mCurrentRotateState;

  public Drivetrain() {
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
    mBackLeft.setInverted(InvertType.None); //REVIEW COMMENT: This should follow master
    mFrontRight.setInverted(InvertType.InvertMotorOutput);
    mBackRight.setInverted(InvertType.InvertMotorOutput); //REVIEW COMMENT: This should follow master

    //REVIEW COMMENT: I dont think we need these states
    mCurrentState = FrontState.FORWARD;
    mCurrentRotateState = RotateState.POSITVIE;

    mDifferentialDrive = new DifferentialDrive(mFrontLeft, mFrontRight);

    //REVIEW COMMENT: Reminder to look at current limmit again with a mentor
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 40, 40, 0);
    mFrontLeft.configSupplyCurrentLimit(currentLimit);
    mFrontRight.configSupplyCurrentLimit(currentLimit);
    mBackLeft.configSupplyCurrentLimit(currentLimit);
    mBackRight.configSupplyCurrentLimit(currentLimit);
  }

  public void drive(double xSpeed, double rSpeed, boolean turnInPlace) {
    mDifferentialDrive.arcadeDrive(
        xSpeed * mCurrentState.direction, -rSpeed * mCurrentState.direction, false);
  }

  //REVIEW COMMENT: Not sure what this is for
  public void forward(double speed) {
    mFrontLeft.set(speed);
    mFrontRight.set(speed);
  }

  public Command changeState(FrontState frontState) {
    return new InstantCommand(() -> mCurrentState = frontState);
  }

  public Command changeRotateState(RotateState RotateState) {
    return new InstantCommand(() -> mCurrentRotateState = RotateState);
  }

  @Override
  public void periodic() {
    //REVIEW COMMENT: These should be logged instead. Use Logger.recordOutput()
    SmartDashboard.putNumber("Front Left Motor", mFrontLeft.get());
    SmartDashboard.putNumber("Back Left Motor", mBackLeft.get());
    SmartDashboard.putNumber("Front Right Motor", mFrontRight.get());
    SmartDashboard.putNumber("Back Right Motor", mBackRight.get());
    SmartDashboard.putNumber("Foward", mCurrentState.direction);
    SmartDashboard.putNumber("Turn", mCurrentRotateState.direction);
  }
}
