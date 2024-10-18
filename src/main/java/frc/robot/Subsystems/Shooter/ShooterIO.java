package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;

public interface ShooterIO {
    

    @AutoLog
  public static class ShooterIOInputs {
    public double flywheelMotorVoltageL = 0.0; // Volts
    public double flywheelMotorVoltageR = 0.0; // Volts

    public double flywheelVelocityL = 0.0; // RPM
    public double flywheelVelocityR = 0.0; // RPM

    public FlywheelSetPoint flywheelSetPoint = new FlywheelSetPoint(0,0); 
    public boolean flywheelAtSetPointL = false;
    public boolean flywheelAtSetPointR = false;

    public double flywheelMotorCurrentL = 0.0; // Amps
    public double flywheelMotorCurrentR = 0.0; // Amps

    public double TOF_Distance = 0;

    //public double flywheelMotorTemp = 0.0; // Celcius
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void changeRPMTarget(FlywheelSetPoint Targets) {}


}
