package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    

    @AutoLog
  public static class ShooterIOInputs {
    public double flywheelMotorVelocity = 0.0; // RPM
    public double flywheelMotorTemp = 0.0; // Celcius
    public double flywheelMotorVoltage = 0.0; // Volts
    public double flywheelMotorCurrent = 0.0; // Amps
    public boolean flywheelAtSetpoint = false; // I don't think this is necessary for a flywheel, I will probably delete it later
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void changeVelocityTarget(double velocityTarget) {}//Might want to change the variable name from "velocityTarget" to something else like "rpmTarget"


}
