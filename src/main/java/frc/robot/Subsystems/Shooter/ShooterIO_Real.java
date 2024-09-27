

package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.RobotConstants.CANID;

public class ShooterIO_Real implements ShooterIO {






// Flywheel Motor Controllers
TalonFX followerFlywheel = new TalonFX(CANID.kRightFlywheel);
TalonFX leaderFlywheel = new TalonFX(CANID.kLeftFlywheel);




// Variables to store/log the setpoints
@AutoLogOutput(key = "Outtake/RPM Setpoint")
private double rpmSetpoint = 0;

private VelocityVoltage flywheelSetpoint = new VelocityVoltage(0).withSlot(0);














    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        //this coresponds to the autologged inputs
    }




}
