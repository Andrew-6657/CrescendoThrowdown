package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    

    @AutoLog
  public static class ShooterIOInputs {
    public double flywheelMotorVelocity = 0.0; // RPM
    public double flywheelMotorTemp = 0.0; // Celcius
    public double flywheelMotorVoltage = 0.0; // Volts
    public double flywheelMotorCurrent = 0.0; // Amps
    public boolean flywheelAtSetpoint = false;
  }



}
