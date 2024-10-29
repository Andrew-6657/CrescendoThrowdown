// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Subsystems.Pivot.Pivot;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.Vision;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  // Driver Controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  // Subsystems
  private Drivetrain drivetrain = new Drivetrain();
  private Pivot pivot = new Pivot();
  private Shooter shooter = new Shooter(null);

  // Vision
  private Vision vision = new Vision();

  // AKit Setup
  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;

  @Override
  public void robotInit() {
    Logger.recordMetadata("Codebase", "6657 2024 Offseason");
    switch (mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }

    SignalLogger.enableAutoLogging(false);

    Logger.start();

    // Driver Controls

    driver.rightTrigger().whileTrue(
      Commands.sequence(
        Commands.parallel(
          shooter.changeSetpoint(ShooterConstants.kSpeaker),
          pivot.aimWithVision(vision::getVisionFrame),
          drivetrain.speakerAlign(vision::getVisionFrame)).raceWith(
            Commands.waitUntil(
              (BooleanSupplier) Commands.parallel(
                Commands.waitUntil(shooter::atSetpoint),
                Commands.waitUntil(pivot::atSetpoint),
                Commands.waitUntil(drivetrain::isAligned)
                )
          )
        ),
        Commands.runOnce( () -> {
          shooter.changeKickerSetPoint(1);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

    driver.rightTrigger().onFalse(Commands.runOnce(() -> {
        shooter.changeSetpoint(ShooterConstants.kIdle);
        pivot.changeSetpoint(PivotConstants.minimumPosition);
    }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    

    driver.leftTrigger().whileTrue(drivetrain.speakerAlign(vision::getVisionFrame));

    operator.a().onTrue(pivot.changeSetpoint(0));

    operator.povDown().onTrue(pivot.incrementSetpoint(-0.3));
    operator.povUp().onTrue(pivot.incrementSetpoint(0.3));

    operator.b().onTrue(shooter.changeSetpoint(ShooterConstants.kStop));
  }

  @Override
  public void robotPeriodic() {
    vision.updateVisionData();
  }

  @Override
  public void disabledInit() {
    if (pivot.getAngle() < 80) pivot.EnableCoastMode();
  }
}
