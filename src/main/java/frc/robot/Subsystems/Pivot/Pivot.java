// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// note, pivote gearing is 7 * 5 * 8

package frc.robot.Subsystems.Pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

  private PIDController pivotePID = new PIDController(0, 0, 0);

  private double pivotSetpoint = 0; //This should be private, we dont want anything changing it outside of the changeSetpoint method.

  private WPI_TalonSRX pivotMotorL = new WPI_TalonSRX(CANID.kPivotL); 
  private WPI_TalonSRX pivotMotorR = new WPI_TalonSRX(CANID.kPivotR); 
  //This should also be WPI_TalonSRX and there is a left and right one. One should inversely follow the other.

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0); //Most things up here should be private since hardware should be kept within the subsystem and not messed with from outside.
  
  public Pivot() {
    //This whole next block probably need modified since were not using a talonFX.
    /* 
    var pivotConfigurator = pivotMotorL.getConfigurator();
        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.Feedback.SensorToMechanismRatio =
            1.0 / PivotConstants.kGearing; // Sets default output to pivot rotations
        pivotConfigs.Slot0 = PivotConstants.kSlot0; // PID Constants
        pivotConfigs.CurrentLimits = PivotConstants.kPivotCurrentConfigs; // Current Limits
        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigurator.apply(pivotConfigs);    */

    //var pivotConfigurator = pivotMotorL.

    pivotEncoder.setPositionOffset(PivotConstants.posOffset);

    

  }

  public double readEncoderValue(){
    return pivotEncoder.getAbsolutePosition()-pivotEncoder.getPositionOffset();
  }

  @Override
  public void periodic() {
    //double output = pivotePID.calculate(pivotEncoder.getAbsolutePosition, pivoteSetpoint);

    //Set motor powers here
  }
}
