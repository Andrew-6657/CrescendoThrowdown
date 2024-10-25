// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// note, pivote gearing is 7 * 5 * 8

package frc.robot.Subsystems.Pivot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.VisionFrame;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private PIDController pivotPID = new PIDController(0, 0, 0);

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  private double pivotSetPoint = 0;

  private WPI_TalonSRX pivotMotorL = new WPI_TalonSRX(CANID.kPivotL);
  private WPI_TalonSRX pivotMotorR = new WPI_TalonSRX(CANID.kPivotR);

  private InterpolatingDoubleTreeMap pivotMap;

  public Pivot() {

    pivotMotorL.configFactoryDefault();
    pivotMotorR.configFactoryDefault();

    pivotMotorL.setNeutralMode(NeutralMode.Brake);
    pivotMotorL.configVoltageCompSaturation(10); // tune later
    pivotMotorL.enableVoltageCompensation(true);
    pivotMotorL.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));

    pivotMotorR.setNeutralMode(NeutralMode.Brake);
    pivotMotorR.configVoltageCompSaturation(10); // tune later
    pivotMotorR.enableVoltageCompensation(true);
    pivotMotorR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));

    pivotMotorL.follow(pivotMotorR);

    pivotMotorL.setInverted(InvertType.OpposeMaster);
    pivotMotorR.setInverted(InvertType.InvertMotorOutput);

    pivotEncoder.setPositionOffset(PivotConstants.posOffset);

    // pivotMap.put(0d, 0d);
  }

  public Command changeSetpoint(double setPoint) {
    return Commands.runOnce(
        () -> {
          pivotSetPoint =
              MathUtil.clamp(
                  setPoint, PivotConstants.minimumPosition, PivotConstants.maximumPosition);
        },
        this);
  }

  public Command relativeChangeSetpoint(double difference){
    return Commands.run(
      () -> {
        changeSetpoint(pivotSetPoint + difference);
      });
  }

  /* 
  public Command freezeSetpoint(){
    return Commands.run(
      () -> {
        changeSetpoint(readEncoderValue());
      }
    );
  } //this should change the setpoint to wherever the pivot already is so the pivot will stop moving */

   
  public Command setPivotAngleFromVision(Supplier<VisionFrame> visionFrameSupplier) {
    return Commands.run(
        () -> {
          VisionFrame visionFrame = visionFrameSupplier.get();
          pivotSetPoint = (visionFrame.hasTarget) ? pivotMap.get(visionFrame.tY) : 0;
        });
  }
  

  public double readEncoderValue() {
    return pivotEncoder.getAbsolutePosition() - pivotEncoder.getPositionOffset();
  }

  public boolean pivotAtSetPoint() {
    return MathUtil.isNear(pivotSetPoint, readEncoderValue(), PivotConstants.setPointTolerance);
  }

  @Override
  public void periodic() {
    double output = pivotPID.calculate(readEncoderValue(), pivotSetPoint);
    pivotMotorR.setVoltage(output);

    Logger.recordOutput("Pivot/Output", output);
    Logger.recordOutput("Pivot/Set Point", pivotSetPoint);

    Logger.recordOutput("Pivot/Encoder ", readEncoderValue());
    Logger.recordOutput("Pivot/At Set Point", pivotAtSetPoint());
  }
}
