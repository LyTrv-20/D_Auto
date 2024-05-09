// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinder;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.HockeyStop;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.drivetrain.XPattern;
import frc.robot.commands.AutoConfigs;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class RobotContainer {

  public static CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  public static CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);
  public static CSP_Controller test = new CSP_Controller(2);

  Swerve drive = Swerve.getInstance();
  Sensors sensors = Sensors.getInstance();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private Notifier shuffleUpdater = new Notifier(() -> updateShuffle());

  public RobotContainer() {
    // Set the default commands
    setDefaultCommands();

    smartdashboardButtons();

    configureBindings();

    shuffleUpdater.startPeriodic(0.02);

    NamedCommands.registerCommands(AutoConfigs.EVENTS);

    addChooser();
  }

  private void setDefaultCommands() {
    drive.setDefaultCommand(new XPattern());
  }

  private void configureBindings() {
    Trigger drivingInput = new Trigger(() -> (pilot.getCorrectedLeft().getNorm() != 0.0 || pilot.getCorrectedRight().getX() != 0.0));
    drivingInput
    .onTrue(    
      new TeleDrive(
        () -> pilot.getCorrectedLeft().getX() * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
        () -> pilot.getCorrectedLeft().getY() * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
        () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.1 : 1.0)))
    .onFalse(new HockeyStop().withTimeout(0.5));

    pilot
        .getStartButton()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.resetOdometry(
                    new Pose2d(drive.getPose2d().getTranslation(), 
                    Rotation2d.fromDegrees(sensors.getAllianceColor() == DriverStation.Alliance.Red ? 180 : 0)));
                  drive.rotPID.setSetpoint(180.0);
                }, sensors));
  }

  public void updateShuffle() {

    
  }

  public void smartdashboardButtons() {

  }

  public void addChooser() {
    autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup());
    autoChooser.addOption("path find (plesase work)", AutoBuilder.pathfindToPose(
      new Pose2d(10, 5, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720)
      ),
      0, 0
      ));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
