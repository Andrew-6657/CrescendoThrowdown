package frc.robot.Subsystems.Shooter;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;

public class ShooterIO_Real implements ShooterIO {

  // Intake TOF Sensor
  LaserCan sensor = new LaserCan(CANID.kIntakeTOF);

  // Flywheel Motor Controllers
  TalonFX rightFlywheel = new TalonFX(CANID.kRightFlywheel);
  TalonFX leftFlywheel = new TalonFX(CANID.kLeftFlywheel);

  // Variables to store/log the setpoints
  private FlywheelSetPoint flywheelSetPoint = new FlywheelSetPoint(0, 0);

  private VelocityVoltage leftWheelSetpoint = new VelocityVoltage(0).withSlot(0);
  private VelocityVoltage rightWheelSetpoint = new VelocityVoltage(0).withSlot(0);

  // REVIEW COMMENT: This can be replaced by a LaserCAN. See: Stumpy intake
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

  // REVIEW COMMENT: variable name should be newSetpoints
  @Override
  public void changeRPMTarget(FlywheelSetPoint Targets) {
    flywheelSetPoint = Targets;
  }

  public boolean flywheelsAtSetPoint() {
    return MathUtil.isNear(
            flywheelSetPoint.leftRPM,
            leftFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.LeftFlywheels.setPointTolerance)
        && MathUtil.isNear(
            flywheelSetPoint.rightRPM,
            rightFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.RightFlywheels.setPointTolerance);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.flywheelMotorVoltageL = leftFlywheel.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.flywheelMotorVoltageR = rightFlywheel.getMotorVoltage().getValueAsDouble(); // Volts

    inputs.flywheelMotorCurrentL = leftFlywheel.getSupplyCurrent().getValueAsDouble(); // Amps
    inputs.flywheelMotorCurrentR = rightFlywheel.getSupplyCurrent().getValueAsDouble(); // Amps

    inputs.flywheelSetPointL = flywheelSetPoint.leftRPM;
    inputs.flywheelSetPointR = flywheelSetPoint.rightRPM;

    double leftVelocity = leftFlywheel.getVelocity().getValueAsDouble() * 60; // RPM
    double rightVelocity = rightFlywheel.getVelocity().getValueAsDouble() * 60; // RPM

    inputs.flywheelVelocityL = leftVelocity; // RPM
    inputs.flywheelVelocityR = rightVelocity; // RPM

    inputs.flywheelAtSetPointL =
        MathUtil.isNear(
            flywheelSetPoint.leftRPM,
            leftFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.LeftFlywheels.setPointTolerance);
    inputs.flywheelAtSetPointR =
        MathUtil.isNear(
            flywheelSetPoint.rightRPM,
            rightFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.RightFlywheels.setPointTolerance);

    leftFlywheel.setControl(
        leftWheelSetpoint
            .withVelocity(flywheelSetPoint.leftRPM / 60)
            .withSlot(0)); // RPM to Native Rotations per second

    rightFlywheel.setControl(
        rightWheelSetpoint
            .withVelocity(flywheelSetPoint.rightRPM / 60)
            .withSlot(0)); // RPM to Native Rotations per second

    inputs.TOF_Distance = Units.metersToInches(sensor.getMeasurement().distance_mm * 0.001);
  }
}
