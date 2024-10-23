// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;

public class Shooter extends SubsystemBase {

  // IO Definition
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  /** Creates a new Shooter. */
  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  public Command changeFlywheelSetpoint(FlywheelSetPoint flywheelSetPoint) {
    return this.runOnce(() -> shooterIO.newSetpoints(flywheelSetPoint));
  }



  public boolean atFlywheelSetpoint() {
    return shooterInputs.flywheelAtSetPointL && shooterInputs.flywheelAtSetPointR;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
