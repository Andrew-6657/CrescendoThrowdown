// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// note, pivote gearing is 7 * 5 * 8

package frc.robot.Subsystems.Pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants.CANID;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  public Pivot() {

    double pivotSetpoint = 0;

    // Pivot Motor Controller
    TalonFX pivotMotor = new TalonFX(CANID.kPivot);

    var pivotConfigurator = pivotMotor.getConfigurator();
        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.Feedback.SensorToMechanismRatio =
            1.0 / PivotConstants.kGearing; // Sets default output to pivot rotations
        pivotConfigs.Slot0 = PivotConstants.kSlot0; // PID Constants
        pivotConfigs.CurrentLimits = PivotConstants.kPivotCurrentConfigs; // Current Limits
        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigurator.apply(pivotConfigs);    

    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
    pivotEncoder.setPositionOffset(PivotConstants.posOffset);
    double pivotEncoderValue = pivotEncoder.getAbsolutePosition()-pivotEncoder.getPositionOffset();

  }

private PIDController pivotePID = new PIDController(0, 0, 0);

  @Override
  public void periodic() {
    double output = pivotePID.calculate(pivotEncoder.getAbsolutePosition, pivoteSetpoint);
  }
}
