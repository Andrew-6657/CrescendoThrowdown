// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  public Command changeSetpoint(FlywheelSetPoint flywheelSetPoint) {
    return this.runOnce(() -> shooterIO.changeSetpoint(flywheelSetPoint));
  }

  public Command changeKickerSetPoint(double setpoint) {
    return this.runOnce(() -> shooterIO.changeKickerSetPoint(setpoint));
  }

  public boolean atSetpoint() {
    return shooterInputs.leftAtSetPoint && shooterInputs.rightAtSetPoint;
  }

  @AutoLogOutput(key = "Shooter/NoteDetected")
  public boolean noteDetected() {
    return (shooterInputs.tofDistance < 1);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }
}
