// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Pivot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.VisionFrame;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private WPI_TalonSRX leftMotor = new WPI_TalonSRX(CANID.kPivotL);
  private WPI_TalonSRX rightMotor = new WPI_TalonSRX(CANID.kPivotR);

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
  private PIDController anglePID = new PIDController(0, 0, 0); // TODO: Tune

  private double setPoint = 0; // degrees I think
  private InterpolatingDoubleTreeMap pivotMap;

  public Pivot() {

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, PivotConstants.kCurrentLimit, PivotConstants.kCurrentLimit, 0));

    rightMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, PivotConstants.kCurrentLimit, PivotConstants.kCurrentLimit, 0));

    leftMotor.follow(rightMotor);

    leftMotor.setInverted(InvertType.OpposeMaster);
    rightMotor.setInverted(InvertType.InvertMotorOutput);

    pivotEncoder.setPositionOffset(PivotConstants.encoderOffset);

    pivotMap.put(0d, 0d);
  }

  public Command EnableCoastMode() {
    return Commands.runOnce(
        () -> {
          rightMotor.setNeutralMode(NeutralMode.Coast);
          leftMotor.setNeutralMode(NeutralMode.Coast);
        },
        this);
  }

  /**
   * Changes the current pivot setpoint
   *
   * @param newSetpoint The setpoint to change to.
   * @return Command to run
   */
  public Command changeSetpoint(double newSetpoint) {
    return Commands.runOnce(
        () -> {
          setPoint =
              MathUtil.clamp(
                  newSetpoint, PivotConstants.minimumPosition, PivotConstants.maximumPosition);
        },
        this);
  }

  /**
   * Increment the current setpoint by a given value.
   *
   * @param difference The amount to change the setpoint by.
   * @return Command to run
   */
  public Command incrementSetpoint(double difference) {
    return Commands.runOnce(
        () -> {
          changeSetpoint(setPoint + difference);
        });
  }

  /**
   * Changes the setpoint of the shooter depending on vision data If tracking is lost mid command,
   * the setpoint is left unchanged.
   *
   * @param visionFrameSupplier Method that returns a VisionFrame object
   * @return Command to run
   */
  public Command aimWithVision(Supplier<VisionFrame> visionFrameSupplier) {
    return Commands.run(
        () -> {
          VisionFrame visionFrame = visionFrameSupplier.get();
          setPoint = (visionFrame.hasTarget) ? pivotMap.get(visionFrame.tY) : setPoint;
        });
  }

  /**
   * @return Current angle reported by the pivot encoder.
   */
  public double getAngle() {
    return pivotEncoder.getAbsolutePosition() - pivotEncoder.getPositionOffset();
  }

  /**
   * @return True if the pivot is at its setpoint within a specified tolerance.
   */
  public boolean atSetpoint() {
    return MathUtil.isNear(setPoint, getAngle(), PivotConstants.positionTolerance);
  }

  @Override
  public void periodic() {
    // Runs the Pivot PID
    double output = anglePID.calculate(getAngle(), setPoint);

    /*
    if (atSetpoint() && setPoint == 0) {
      rightMotor.setVoltage(0);
    } else {
      rightMotor.setVoltage(output);
    } */

    // Logging
    Logger.recordOutput("Pivot/Output", output);
    Logger.recordOutput("Pivot/Current", leftMotor.getSupplyCurrent());
    Logger.recordOutput("Pivot/Set Point", setPoint);
    Logger.recordOutput("Pivot/Angle", getAngle());
    Logger.recordOutput("Pivot/At Setpoint", atSetpoint());
  }
}
