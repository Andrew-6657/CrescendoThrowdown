// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// note, pivote gearing is 7 * 5 * 8

package frc.robot.Subsystems.Pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;

public class Pivot extends SubsystemBase {

  private PIDController pivotPID = new PIDController(0, 0, 0);

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  private double pivotSetpoint = 0; //This should be private, we dont want anything changing it outside of the changeSetpoint method.

  private WPI_TalonSRX pivotMotorL = new WPI_TalonSRX(CANID.kPivotL); 
  private WPI_TalonSRX pivotMotorR = new WPI_TalonSRX(CANID.kPivotR); 
  //This should also be WPI_TalonSRX and there is a left and right one. One should inversely follow the other.

  public Pivot() {

    pivotMotorL.configFactoryDefault();
    pivotMotorR.configFactoryDefault();

    pivotMotorL.setNeutralMode(NeutralMode.Brake);
    pivotMotorL.configVoltageCompSaturation(10); //tune later
    pivotMotorL.enableVoltageCompensation(true);
    pivotMotorL.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0)); //ask Andy or someone what triggerThresholdCurrent does

    pivotMotorR.setNeutralMode(NeutralMode.Brake);
    pivotMotorR.configVoltageCompSaturation(10); //tune later
    pivotMotorR.enableVoltageCompensation(true);
    pivotMotorR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0)); //ask Andy or someone what triggerThresholdCurrent does

    pivotMotorL.setInverted(false);
    pivotMotorR.setInverted(true);

    pivotMotorL.follow(pivotMotorR);


    pivotEncoder.setPositionOffset(PivotConstants.posOffset);

    

  }

  public void changeSetpoint(double setPoint) {
    pivotSetpoint = MathUtil.clamp(setPoint, PivotConstants.minimumPosition, PivotConstants.maximumPosition);//the min and max need to be calculated
  }

  public double readEncoderValue(){
    //return pivotEncoder.getAbsolutePosition()-pivotEncoder.getPositionOffset();
    return .0;
  }

  @Override
  public void periodic() {
    //double output = pivotePID.calculate(pivotEncoder.getAbsolutePosition, pivoteSetpoint);

    // Set motor powers here
  }
}
