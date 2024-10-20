package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterIO_Real implements ShooterIO {

  // Flywheel Motor Controllers
  TalonFX rightFlywheel = new TalonFX(CANID.kRightFlywheel);
  TalonFX leftFlywheel = new TalonFX(CANID.kLeftFlywheel);

  // Variables to store/log the setpoints
  @AutoLogOutput(key = "Outtake/RPM Setpoint")
  private FlywheelSetPoint flywheelSetPoint = new FlywheelSetPoint(0, 0);

  private VelocityVoltage rightWheelSetpoint = new VelocityVoltage(0).withSlot(0);
  private VelocityVoltage leftWheelSetpoint = new VelocityVoltage(0).withSlot(0);

  // Chamber Beam Break Sensor
  DigitalInput beambreak = new DigitalInput(8);

  public ShooterIO_Real() {

    // Configure the flywheel motor controllers
    var rightFlywheelConfigurator = rightFlywheel.getConfigurator();
    var rightFlywheelConfig = new TalonFXConfiguration();
    rightFlywheelConfig.Feedback.SensorToMechanismRatio =
        1.0 / ShooterConstants.RightFlywheels.kGearing; // Sets default output to flywheel rotations
    rightFlywheelConfig.Slot0 = ShooterConstants.RightFlywheels.kSlot0; // PID Constants
    rightFlywheelConfig.CurrentLimits =
        ShooterConstants.RightFlywheels.kCurrentConfigs; // Current Limits
    rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightFlywheelConfigurator.apply(rightFlywheelConfig);

    var leftFlywheelConfigurator = leftFlywheel.getConfigurator();
    var leftFlywheelConfig = new TalonFXConfiguration();
    leftFlywheelConfig.Feedback.SensorToMechanismRatio =
        1.0 / ShooterConstants.LeftFlywheels.kGearing; // Sets default output to flywheel rotations
    leftFlywheelConfig.Slot0 = ShooterConstants.LeftFlywheels.kSlot0; // PID Constants
    leftFlywheelConfig.CurrentLimits =
        ShooterConstants.LeftFlywheels.kCurrentConfigs; // Current Limits
    leftFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftFlywheelConfigurator.apply(leftFlywheelConfig);
  }

  @Override
  public void changeRPMTarget(FlywheelSetPoint Targets) {
    flywheelSetPoint = Targets;
  }

  // would this make changing the set point look like "changeRPMTarget(new
  // FlywheelSetPoint(100,200));" or "changeRPMTarget(ShooterConstants.kSpeakerRPM);" ?

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // public double flywheelMotorVoltageL = 0.0; // Volts
    // public double flywheelMotorVoltageR = 0.0; // Volts

    // public double flywheelVelocityL = 0.0; // RPM
    // public double flywheelVelocityR = 0.0; // RPM

    // public FlywheelSetPoint flywheelSetPoint = new FlywheelSetPoint(0,0);
    // public boolean flywheelAtSetPointL = false;
    // public boolean flywheelAtSetPointR = false;

    // public double flywheelMotorCurrentL = 0.0; // Amps
    // public double flywheelMotorCurrentR = 0.0; // Amps

    // public double TOF_Distance = 0;
    // this coresponds to the autologged inputs
  }
}
