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

    public static FlywheelSetPoint flywheelSetPoint = new FlywheelSetPoint(0,0); 
    public boolean flywheelAtSetpointL = false;
    public boolean flywheelAtSetpointR = false;

    public double flywheelMotorCurrentL = 0.0; // Amps
    public double flywheelMotorCurrentR = 0.0; // Amps

    public double TOF_Distance = 0;

    //public double flywheelMotorTemp = 0.0; // Celcius
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void changeVelocityTarget(double leftFlywheel, double rightFlywheel) {}//Might want to change the variable name from "velocityTarget" to something else like "rpmTarget"


}
