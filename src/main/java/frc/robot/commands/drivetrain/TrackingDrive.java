package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class TrackingDrive extends Command {
  private Swerve drive = Swerve.getInstance();

  DoubleSupplier xInput, yInput, goalAngle;
  boolean noInput;
  PIDController rotPID = drive.rotPID;

  /** Creates a new TrackingDrive. */
  public TrackingDrive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier goalAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.xInput = xInput;
    this.yInput = yInput;
    this.goalAngle = goalAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double goalAngleDeg = goalAngle.getAsDouble();
    
    // Pose2d pose = drive.getPose2d();
    // Translation2d currentSpeed = drive.getFOSpeeds();

    // double dx = goal.getX() - pose.getX(); //get distance needed to travel in x
    // double dy = goal.getY() - pose.getY(); //get distance needed to travel in y

    /* Get hypothenuse cube of the slope yInput and xInput
     * cube = scaling 
     * if input not full/max, the output would be closer to 0 for the best driver control
     * if input is rotpid, the output would be max too -> cube makes the transition smoother
     */
    double totalSpeed = Math.pow(Math.hypot(xInput.getAsDouble(), yInput.getAsDouble()), 3.0);
    //arctan of the slope of y and x = angle
    double angle = Math.atan2(yInput.getAsDouble(), xInput.getAsDouble());

    double xSpeed = totalSpeed * Math.cos(angle) * Constants.drivetrain.MAX_VELOCITY;
    double ySpeed = totalSpeed * Math.sin(angle) * Constants.drivetrain.MAX_VELOCITY;
    double rotSpeed = 0.0;

    //rotSpeed calculated from rotPID
    rotSpeed += drive.rotPID.calculate(Swerve.getInstance().getPose2d().getRotation().getDegrees(), goalAngleDeg);
    /**rotSpeed from aiden's math hellscape = predicting wut the drivetrain rotation speed should be
     * to keep aiming while moving */
    // rotSpeed += Math.toDegrees(currentSpeed.getX() * -dy / (dx * dx + dy * dy) + currentSpeed.getY() * dx / (dx * dx + dy * dy));

    noInput = xSpeed == 0 && ySpeed == 0 && rotSpeed == 0;

    //make the swerve form "X" to prevent drifting when stop
    if (noInput) {
      drive.setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
    } else {
      drive.setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
          xSpeed,
          ySpeed, 
          rotSpeed), 
          Sensors.getInstance().getRotation2d()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
