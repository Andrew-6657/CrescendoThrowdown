package frc.robot.Subsystems.Shooter;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;

public class ShooterIO_Real implements ShooterIO {

  // Chamber TOF Sensor
  private LaserCan sensor = new LaserCan(CANID.kChamberTOF);

  // Flywheel Motors
  private TalonFX rightFlywheel = new TalonFX(CANID.kRightFlywheel);
  private TalonFX leftFlywheel = new TalonFX(CANID.kLeftFlywheel);

  private WPI_TalonSRX kicker = new WPI_TalonSRX(CANID.kKicker);
  private double kickerSetPoint = 0.0;

  private FlywheelSetPoint setpoint = new FlywheelSetPoint(0, 0);

  private VelocityVoltage leftVV = new VelocityVoltage(0).withSlot(0);
  private VelocityVoltage rightVV = new VelocityVoltage(0).withSlot(0);

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

    // Configure the kicker motor
    kicker.configFactoryDefault();
    kicker.setNeutralMode(NeutralMode.Brake);
    kicker.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, ShooterConstants.Kicker.kCurrentLimit, ShooterConstants.Kicker.kCurrentLimit, 0));
  }

  @Override
  public void changeKickerSetPoint(double setPoint) {
    kickerSetPoint = setPoint;
  }

  @Override
  public void changeSetpoint(FlywheelSetPoint newSetpoint) {
    setpoint = newSetpoint;
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(
            setpoint.leftRPM,
            leftFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.LeftFlywheels.rpmTolerance)
        && MathUtil.isNear(
            setpoint.rightRPM,
            rightFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.RightFlywheels.rpmTolerance);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.leftVoltage = leftFlywheel.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.rightVoltage = rightFlywheel.getMotorVoltage().getValueAsDouble(); // Volts

    inputs.leftCurrent = leftFlywheel.getSupplyCurrent().getValueAsDouble(); // Amps
    inputs.rightCurrent = rightFlywheel.getSupplyCurrent().getValueAsDouble(); // Amps

    inputs.leftSetPoint = setpoint.leftRPM;
    inputs.rightSetPoint = setpoint.rightRPM;

    double leftVelocity = leftFlywheel.getVelocity().getValueAsDouble() * 60; // RPM
    double rightVelocity = rightFlywheel.getVelocity().getValueAsDouble() * 60; // RPM

    inputs.leftVelocity = leftVelocity; // RPM
    inputs.rightVelocity = rightVelocity; // RPM

    inputs.leftAtSetPoint =
        MathUtil.isNear(
            setpoint.leftRPM,
            leftFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.LeftFlywheels.rpmTolerance);
    inputs.rightAtSetPoint =
        MathUtil.isNear(
            setpoint.rightRPM,
            rightFlywheel.getVelocity().getValueAsDouble() * 60,
            ShooterConstants.RightFlywheels.rpmTolerance);
    /*
        leftFlywheel.setControl(
            leftVV
                .withVelocity(setpoint.leftRPM / 60)
                .withSlot(0)); // RPM to Native Rotations per second

        rightFlywheel.setControl(
            rightVV
                .withVelocity(setpoint.rightRPM / 60)
                .withSlot(0)); // RPM to Native Rotations per second
    */
    inputs.tofDistance = Units.metersToInches(sensor.getMeasurement().distance_mm * 0.001);

    inputs.kickerSetPoint = kickerSetPoint;

    kicker.set(kickerSetPoint);
    inputs.kickerCurrent = kicker.getSupplyCurrent();
  }
}
