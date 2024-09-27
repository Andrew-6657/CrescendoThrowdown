package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    

    @AutoLog
  public static class ShooterIOInputs {
    public double flywheelMotorVoltageL = 0.0; // Volts
    public double flywheelMotorVoltageR = 0.0; // Volts
    public double flywheelMotorVelocityL = 0.0; // RPM
    public double flywheelMotorVelocityR = 0.0; // RPM
    //public double flywheelMotorTemp = 0.0; // Celcius
    public double flywheelMotorCurrent = 0.0; // Amps
    public boolean flywheelAtSetpoint = false; // I don't think this is necessary for a flywheel, I will probably delete it later. Maybe we should change it to "targetSpeed"?

    public double TOF_Distance = 0;// I feal like this is supposed to be somewhere else, but I think this is where it was in the main robot
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void changeVelocityTarget(double velocityTarget) {}//Might want to change the variable name from "velocityTarget" to something else like "rpmTarget"


}
