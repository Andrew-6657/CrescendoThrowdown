// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// note, pivote gearing is 7 * 5 * 8

package frc.robot.Subsystems.Pivot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
=======
>>>>>>> 8757b985cbbab9752e1e4166233735b60238c514
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants.CANID;

public class Pivot extends SubsystemBase {

<<<<<<< HEAD
  private PIDController pivotePID = new PIDController(0, 0, 0);

  private double pivotSetpoint = 0; //This should be private, we dont want anything changing it outside of the changeSetpoint method.

  private WPI_TalonSRX pivotMotorL = new WPI_TalonSRX(CANID.kPivotL); 
  private WPI_TalonSRX pivotMotorR = new WPI_TalonSRX(CANID.kPivotR); 
  //This should also be WPI_TalonSRX and there is a left and right one. One should inversely follow the other.
=======
  private double pivotSetpoint =
      0; // This should be private, we dont want anything changing it outside of the changeSetpoint
  // method.

  private TalonFX pivotMotor = new TalonFX(CANID.kPivot);
  // Private since nothing else should be able to mess with the motor. This should also be
  // WPI_TalonSRX and there is a left and right one. One should inversely follow the other.

  private DutyCycleEncoder pivotEncoder =
      new DutyCycleEncoder(
          0); // Most things up here should be private since hardware should be kept within the

  // subsystem and not messed with from outside.
>>>>>>> 8757b985cbbab9752e1e4166233735b60238c514

  public Pivot() {
<<<<<<< HEAD
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
=======
    // This whole next block probably need modified since were not using a talonFX.
    var pivotConfigurator = pivotMotor.getConfigurator();
    var pivotConfigs = new TalonFXConfiguration();
    pivotConfigs.Feedback.SensorToMechanismRatio =
        1.0 / PivotConstants.kGearing; // Sets default output to pivot rotations
    pivotConfigs.Slot0 = PivotConstants.kSlot0; // PID Constants
    pivotConfigs.CurrentLimits = PivotConstants.kPivotCurrentConfigs; // Current Limits
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfigurator.apply(pivotConfigs);

    pivotEncoder.setPositionOffset(PivotConstants.posOffset);

    // Make a method to return this value instead. Having it here doesnt really do anything
    double pivotEncoderValue =
        pivotEncoder.getAbsolutePosition() - pivotEncoder.getPositionOffset();
  }

  // This is good just put it up at the top of the file above the constuctor with the rest of the
  // stuff.
  private PIDController pivotePID = new PIDController(0, 0, 0);
>>>>>>> 8757b985cbbab9752e1e4166233735b60238c514

  @Override
  public void periodic() {
    //double output = pivotePID.calculate(pivotEncoder.getAbsolutePosition, pivoteSetpoint);

    // Set motor powers here
  }
}
