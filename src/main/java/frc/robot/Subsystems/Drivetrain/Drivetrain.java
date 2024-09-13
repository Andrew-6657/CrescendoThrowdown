//Most of this is just copypasted from Fern's drivetrain

package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.DriveConstants.FrontState;//I don't know if we want to use front state / rotate state yet
import frc.robot.Constants.DriveConstants.RotateState;
import frc.robot.Constants.RobotConstants.CAN;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX mFrontLeft;
  private final WPI_TalonSRX mFrontRight;
  private final WPI_TalonSRX mBackLeft;
  private final WPI_TalonSRX mBackRight;
  private final WPI_PigeonIMU mPigeon;

  private final DifferentialDrive mDifferentialDrive;
  private FrontState mCurrentState;
  private RotateState mCurrentRotateState;

  public Drivetrain() {
    mFrontLeft = new WPI_TalonSRX(CAN.kFrontLeft);
    mFrontRight = new WPI_TalonSRX(CAN.kFrontRight);
    mBackLeft = new WPI_TalonSRX(CAN.kBackLeft);
    mBackRight = new WPI_TalonSRX(CAN.kBackRight);
    mPigeon = new WPI_PigeonIMU(CAN.kPigeon);

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
    mBackLeft.setInverted(InvertType.None);
    mFrontRight.setInverted(InvertType.InvertMotorOutput);
    mBackRight.setInverted(InvertType.InvertMotorOutput);
    //Eventualy double check that the correct motors are being inverted.


    mCurrentState = FrontState.FORWARD;
    mCurrentRotateState = RotateState.POSITVIE;

    mDifferentialDrive = new DifferentialDrive(mFrontLeft, mFrontRight);

    //Reminder to look at current limmit again with a mentor
    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 30, 40, 0);
    mFrontLeft.configSupplyCurrentLimit(currentLimit);
    mFrontRight.configSupplyCurrentLimit(currentLimit);
    mBackLeft.configSupplyCurrentLimit(currentLimit);
    mBackRight.configSupplyCurrentLimit(currentLimit);
  }


  public Rotation2d getAngle(){
    return mPigeon.getRotation2d().times(-1);
  }


  public void drive(double xSpeed, double rSpeed, boolean turnInPlace){
    mDifferentialDrive.curvatureDrive(xSpeed * mCurrentState.direction, -rSpeed * mCurrentState.direction, turnInPlace);
  }


  public void forward(double speed){
    mFrontLeft.set(speed);
    mFrontRight.set(speed);
  }


  public void resetGyro(){
    mPigeon.reset();
  }


  public Command changeState(FrontState frontState){
    return new InstantCommand(() -> mCurrentState = frontState);
  }

  public Command changeRotateState(RotateState RotateState){
    return new InstantCommand(() -> mCurrentRotateState = RotateState);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Motor", mFrontLeft.get());
    SmartDashboard.putNumber("Back Left Motor", mBackLeft.get());
    SmartDashboard.putNumber("Front Right Motor", mFrontRight.get());
    SmartDashboard.putNumber("Back Right Motor", mBackRight.get());
    SmartDashboard.putNumber("Foward", mCurrentState.direction);
    SmartDashboard.putNumber("Turn", mCurrentRotateState.direction);
  }
}
