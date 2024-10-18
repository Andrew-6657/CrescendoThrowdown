

package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.RobotConstants.CANID;


public class ShooterIO_Real implements ShooterIO {






// Flywheel Motor Controllers
TalonFX rightFlywheel = new TalonFX(CANID.kRightFlywheel);
TalonFX leftFlywheel = new TalonFX(CANID.kLeftFlywheel);




// Variables to store/log the setpoints
@AutoLogOutput(key = "Outtake/RPM Setpoint")
private double rpmSetpoint = 0;

private VelocityVoltage rightWheelSetpoint = new VelocityVoltage(0).withSlot(0);
private VelocityVoltage leftWheelSetpoint = new VelocityVoltage(0).withSlot(0);

// Chamber Beam Break Sensor
  DigitalInput beambreak = new DigitalInput(8);






public ShooterIO_Real() {

    // Motor Controller Configurations

    

    

    

    // Configure the leading flywheel motor
    var rightFlywheelConfigurator = rightFlywheel.getConfigurator();
    var rightFlywheelConfig = new TalonFXConfiguration();
    rightFlywheelConfig.Feedback.SensorToMechanismRatio =
        1.0 / ShooterConstants.RightFlywheels.kGearing; // Sets default output to flywheel rotations
    rightFlywheelConfig.Slot0 = ShooterConstants.RightFlywheels.kSlot0; // PID Constants
    rightFlywheelConfig.CurrentLimits = ShooterConstants.RightFlywheels.kCurrentConfigs; // Current Limits
    rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightFlywheelConfigurator.apply(rightFlywheelConfig);

    
  }





    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        //this coresponds to the autologged inputs
    }




}
