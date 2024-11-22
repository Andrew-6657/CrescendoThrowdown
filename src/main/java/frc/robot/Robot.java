// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetPoint;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Subsystems.Pivot.Pivot;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO_Real;
import frc.robot.Subsystems.Vision.Vision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  // Driver Controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  // Subsystems
  private Drivetrain drivetrain;
  private Pivot pivot = new Pivot();
  private Shooter shooter = new Shooter(new ShooterIO_Real());

  // Vision
  private Vision vision = new Vision();

  // AKit Setup
  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;

  // Auto Command
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  // public Command shootingSequence() {
  //   return Commands.sequence(
  //       Commands.parallel(
  //               shooter.changeSetpoint(ShooterConstants.kSpeaker),
  //               pivot.aimWithVision(vision::getVisionFrame),
  //               drivetrain.speakerAlign(vision::getVisionFrame))
  //           .raceWith(
  //               Commands.parallel(
  //                   Commands.waitUntil(shooter::atSetpoint),
  //                   Commands.waitUntil(pivot::atSetpoint),
  //                   Commands.waitUntil(drivetrain::isAligned))),
  //       Commands.sequence(
  //               shooter.changeKickerSetPoint(1),
  //               Commands.waitUntil(() -> !shooter.noteDetected()),
  //               Commands.waitSeconds(0.3),
  //               Commands.parallel(
  //                       shooter.changeKickerSetPoint(0),
  //                       shooter.changeSetpoint(ShooterConstants.kIdle),
  //                       pivot.changeSetpoint(PivotConstants.minimumPosition))
  //                   .raceWith(
  //                       Commands.parallel(
  //                           Commands.waitUntil(shooter::atSetpoint),
  //                           Commands.waitUntil(pivot::atSetpoint))))
  //           .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  // }

  // public Command spitSequence() {
  //   return Commands.sequence(
  //       Commands.parallel(
  //               shooter.changeSetpoint(ShooterConstants.kSpit),
  //               pivot.changeSetpoint(
  //                   PivotConstants.minimumPosition
  //                       + 5)) // the number of degrees pivot raises from minimum position
  //           .raceWith(
  //               Commands.parallel(
  //                   Commands.waitUntil(shooter::atSetpoint),
  //                   Commands.waitUntil(pivot::atSetpoint))),
  //       shooter.changeKickerSetPoint(1),
  //       Commands.waitUntil(() -> !shooter.noteDetected()),
  //       Commands.waitSeconds(0.3),
  //       Commands.parallel(
  //               shooter.changeKickerSetPoint(0),
  //               shooter.changeSetpoint(ShooterConstants.kIdle),
  //               pivot.changeSetpoint(PivotConstants.minimumPosition))
  //           .raceWith(
  //               Commands.parallel(
  //                   Commands.waitUntil(shooter::atSetpoint),
  //                   Commands.waitUntil(pivot::atSetpoint))));
  // }

  public Command taxi() {
    return Commands.run(() -> drivetrain.drive(0, 0.2, false)).raceWith(Commands.waitSeconds(5));
  }

  @Override
  public void robotInit() {

    drivetrain = new Drivetrain();

    drivetrain.setDefaultCommand(
        Commands.run(
            () -> {
              drivetrain.drive(-driver.getLeftY(), -driver.getRightX(), true);
            },
            drivetrain));

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

    // Autos
    autoChooser.addDefaultOption("None", null);

    driver.povUp().onTrue(pivot.changeSetpoint(45));
    driver.povDown().onTrue(pivot.changeSetpoint(0));

    driver.povRight().onTrue(shooter.changeSetpoint(new FlywheelSetPoint(1000, 2000)));
    driver.povLeft().onTrue(shooter.changeSetpoint(new FlywheelSetPoint(0, 0)));
    // driver.a().onTrue(shooter.changeSetpoint(new FlywheelSetPoint(-2000, -2000)));

    driver.rightTrigger().whileTrue(shooter.changeKickerSetPoint(-1));
    driver.rightTrigger().onFalse(shooter.changeKickerSetPoint(1));

    operator
        .a()
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                    pivot.changeSetpoint(104),
                    Commands.sequence(
                        shooter.changeSetpoint(new FlywheelSetPoint(-3000, -4000)),
                        shooter.changeKickerSetPoint(-0.25))),
                Commands.waitUntil(shooter::noteDetected),
                Commands.parallel(
                    pivot.changeSetpoint(0),
                    Commands.sequence(
                        shooter.changeSetpoint(new FlywheelSetPoint(0, 0)),
                        shooter.changeKickerSetPoint(0)))));

    operator
        .a()
        .onFalse(
            Commands.parallel(
                pivot.changeSetpoint(0),
                Commands.sequence(
                    shooter.changeSetpoint(new FlywheelSetPoint(0, 0)),
                    shooter.changeKickerSetPoint(0))));

    driver
        .b()
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                    pivot.changeSetpoint(65),
                    shooter.changeSetpoint(new FlywheelSetPoint(3000, 6000))),
                Commands.waitSeconds(1.5),
                shooter.changeKickerSetPoint(1),
                Commands.waitSeconds(0.5),
                shooter.changeKickerSetPoint(0),
                shooter.changeSetpoint(ShooterConstants.kIdle),
                pivot.changeSetpoint(0)));

    driver
        .b()
        .onFalse(
            Commands.parallel(
                pivot.changeSetpoint(0),
                Commands.sequence(
                    shooter.changeSetpoint(new FlywheelSetPoint(0, 0)),
                    shooter.changeKickerSetPoint(0))));

    driver
        .y()
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                    pivot.changeSetpoint(55),
                    shooter.changeSetpoint(new FlywheelSetPoint(4000, 8000))),
                Commands.waitSeconds(1.5),
                shooter.changeKickerSetPoint(1),
                Commands.waitSeconds(0.5),
                shooter.changeKickerSetPoint(0),
                shooter.changeSetpoint(ShooterConstants.kIdle),
                pivot.changeSetpoint(0)));

    driver
        .y()
        .onFalse(
            Commands.parallel(
                pivot.changeSetpoint(0),
                Commands.sequence(
                    shooter.changeSetpoint(new FlywheelSetPoint(0, 0)),
                    shooter.changeKickerSetPoint(0))));

    // autoChooser.addOption("ShootPreload", shootingSequence());
    // autoChooser.addOption("Shoot-Taxi", Commands.sequence(shootingSequence(), taxi()));
    // autoChooser.addOption("TaxiOnly", taxi());
    // autoChooser.addOption("SpitOnly", spitSequence());
    // autoChooser.addOption("Taxi-Spit", Commands.sequence(taxi(), spitSequence()));

    // Driver Controls

    // drive

    // Commands.run(() -> drivetrain.drive(0, 0, true)));

    // Shooting at speaker sequence
    // driver.rightTrigger().whileTrue(shootingSequence());

    // driver
    //     .rightTrigger()
    //     .onFalse(
    //         Commands.runOnce(
    //             () -> {
    //               shooter.changeSetpoint(ShooterConstants.kIdle);
    //               pivot.changeSetpoint(PivotConstants.minimumPosition);
    //             }));

    // intake
    // driver
    //     .leftTrigger()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.parallel(
    //                     shooter.changeSetpoint(ShooterConstants.kIntake),
    //                     shooter.changeKickerSetPoint(-1),
    //                     pivot.changeSetpoint(PivotConstants.maximumPosition))
    //                 .raceWith(Commands.waitUntil(shooter::noteDetected)),
    //             Commands.waitSeconds(0.1),
    //             pivot.changeSetpoint(PivotConstants.minimumPosition)));

    // driver
    //     .leftTrigger()
    //     .onFalse(
    //         Commands.sequence(
    //
    // shooter.changeSetpoint(ShooterConstants.kIdle).raceWith(Commands.waitSeconds(0.6)),
    //             shooter.changeKickerSetPoint(-0.25),
    //             pivot.changeSetpoint(PivotConstants.minimumPosition)));

    // driver.a().whileTrue(drivetrain.speakerAlign(vision::getVisionFrame));

    // Operator Controls
    // operator.a().onTrue(pivot.changeSetpoint(0));

    // operator.povDown().onTrue(pivot.incrementSetpoint(-0.3));
    // operator.povUp().onTrue(pivot.incrementSetpoint(0.3));

    // operator.b().onTrue(shooter.changeSetpoint(ShooterConstants.kStop));

    // spit sequence
    // operator.rightTrigger().whileTrue(spitSequence());
  }

  @Override
  public void robotPeriodic() {
    vision.updateVisionData();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    if (pivot.getAngle() < 80) pivot.EnableCoastMode();
  }
}
