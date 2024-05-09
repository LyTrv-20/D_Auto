// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

public class FollowPath extends Command {

  private HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(2.5, 0, 0), 
    new PIDController(2.5, 0, 0), 
    new ProfiledPIDController(0.4 , 0, Constants.drivetrain.ROT_PID.getD(),
    new Constraints(Constants.drivetrain.MAX_RADIANS, 2.5 * Constants.drivetrain.MAX_RADIANS)));

    private final Trajectory trajectory;
    private final Rotation2d heading;
    private final Timer timer = new Timer();
  /** Creates a new Follower. */
  public FollowPath(Trajectory trajectory, Rotation2d heading) {

    addRequirements(Swerve.getInstance());

    this.trajectory = trajectory;
    this. heading = heading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Swerve.getInstance().setChassisSpeeds(controller.calculate(
      Swerve.getInstance().getPose2d(), trajectory.sample(timer.get()), heading));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.getInstance().disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}
