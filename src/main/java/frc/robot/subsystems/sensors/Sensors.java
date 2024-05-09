package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.field.Goal;
import frc.robot.subsystems.drivetrain.Swerve;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  //Offset is 180 because the pigeon is oriented backwards
  private Pigeon pigeon = new Pigeon(Constants.ids.PIGEON, Constants.sensors.pigeon.PIGEON_OFFSET_DEGREES);

  private Limelight limelightFront =
      new Limelight(
          "limelight-front",
          Constants.sensors.limelight.FRONT_LIMELIGHT_LOCATION);

  private Limelight limelightBack =
      new Limelight(
          "limelight-back",
          Constants.sensors.limelight.BACK_LIMELIGHT_LOCATION);

  private Goal currentGoal = Goal.SPEAKER; // NOT USED RIGHT NOW
  private Translation3d speakerLocation = Constants.field.BLUE_SPEAKER_LOCATION;
  private Translation2d cornerLocation = Constants.field.BLUE_CORNER_LOCATION;

  private InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

  /** Creates a new Sensors. */
  private Sensors() { }

  @Override
  public void periodic() {
    setGoals();

    SmartDashboard.putNumber("Speaker Distance", getSpeakerDistance());
    SmartDashboard.putNumber("Corner Distance", getCornerDistance());

    SmartDashboard.putBoolean("Drive Ready?", Swerve.getInstance().atGoalAngle(getFormulaDriveAngle()));

    SmartDashboard.putNumber("Pigeon Rate", getPigeonRate());
    
  }

  public Pose2d getBackPose2d() {
    if (limelightBack.getTV()) return limelightBack.getPose2d();
    return new Pose2d(); 
  }

  public Pose2d getFrontPose2d() {
    if (limelightFront.getTV()) return limelightFront.getPose2d();
    return new Pose2d(); 
  }

  public double getBackLatency() {
    return limelightBack.getLatency();
  }

  public double getFrontLatency() {
    return limelightFront.getLatency();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(pigeon.getRotation());
  }

  public double getPitch() {
    return pigeon.getPitchAsDouble();
  }


  public double getPigeonRate() {
    return pigeon.getRate();
  }

  public void resetPigeon() {
    pigeon.reset();
  }

  public void resetPigeon(Rotation2d rotation) {
    pigeon.reset(rotation);
  }

  public boolean isHappy() {
    Pose2d pose = Swerve.getInstance().getPose2d();
    Pose2d goalPos = currentGoal.position;
    Translation2d goal = goalPos.getTranslation();
    double goalAngle = goalPos.getRotation().getRadians() - Math.PI;

    double angle1 = Math.atan2(
      goal.getX() + Math.cos(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getX(),
      goal.getY() + Math.sin(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getY());
    
    double angle2 = Math.atan2(
      goal.getX() - Math.cos(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getX(),
      goal.getY() - Math.sin(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getY());

    double d1 = (angle1 - angle2 + 2 * (Math.PI / 2.0) % 4 * (Math.PI / 2.0)) - 2 * (Math.PI / 2.0);

    return d1 > currentGoal.happyZone;
  }

  public double getSpeakerDistance() {
    return (speakerLocation.toTranslation2d().getDistance(Swerve.getInstance().getPose2d().getTranslation()) + 0.25) * 0.95;

      //+  (Math.signum(Swerve.getInstance().getPose2d().getRotation().getCos()) > 0.0 ? Units.inchesToMeters(5.5) : 0.0);
  }

  public double getCornerDistance() {
    return cornerLocation.getDistance(Swerve.getInstance().getPose2d().getTranslation()) + 0.25;
  }

  /**
   * Get the RPM that shooter should shoot at based on goal position
   * @param goal
   * @return 
   */
  public double getFormulaShooterRPM() {
    // return currentGoal.ITM_V.get(getXYDistance());
    return velocityMap.get(getSpeakerDistance());
  }

  public double getCornerShooterRPM() {
    return velocityMap.get(getCornerDistance());
  }

  /**
   * Get angle the shoulder should be at while aiming + standing still 
   * @param goal
   * @return angle in double
   */
  public Rotation2d getFormulaShoulderAngle() {
    // return Rotation2d.fromDegrees(currentGoal.ITM_A.get(getXYDistance()));
    return Rotation2d.fromDegrees(-(90.0 - Math.toDegrees(Math.atan(2.04 / getSpeakerDistance())) + angleMap.get(getSpeakerDistance())))
      .times(-Math.signum(Swerve.getInstance().getColorNormRotation().getCos()))

      .plus(Math.signum(Swerve.getInstance().getColorNormRotation().getCos()) > 0.0 ? Rotation2d.fromDegrees(6.0) : Rotation2d.fromDegrees(0.0));
  }

  /** 
   * Get angle the robot / drivetrain should be at while standing still during aimmode
   * @param goal
   * @return angle in  
   */
  public Rotation2d getFormulaDriveAngle() {
    Translation2d translation;
    if(getAllianceColor() == DriverStation.Alliance.Blue) {
      translation = speakerLocation.toTranslation2d().minus(new Translation2d(0.25, 0.0));
    } else {
      translation = speakerLocation.toTranslation2d().plus(new Translation2d(0.25, 0.0));
    }

    Translation2d difference = Swerve.getInstance().getPose2d().getTranslation().minus(translation);

    Rotation2d setpoint = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX())).rotateBy(Rotation2d.fromDegrees(-3.5).times(-Math.signum(Swerve.getInstance().getColorNormRotation().getCos()))); // ROTATION CORRECTION
    //Rotation2d setpoint = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX())).rotateBy(Rotation2d.fromDegrees(-3.5)); // ROTATION CORRECTION

    return Swerve.getInstance().getColorNormRotation().getCos() < 0.0 ? setpoint : setpoint.rotateBy(Rotation2d.fromDegrees(180.0));
  }

  public Rotation2d getCornerDriveAngle() {
    Translation2d translation = Swerve.getInstance().getPose2d().getTranslation().minus(cornerLocation);

    Rotation2d setpoint = Rotation2d.fromRadians(Math.atan2(translation.getY(), translation.getX())).rotateBy(Rotation2d.fromDegrees(-3.5).times(-Math.signum(Swerve.getInstance().getColorNormRotation().getCos()))); // ROTATION CORRECTION
    return Swerve.getInstance().getColorNormRotation().getCos() < 0.0 ? setpoint : setpoint.rotateBy(Rotation2d.fromDegrees(180.0));
  }
  /**
   * Get the vector (angle and speed) for shooting into goal when standing still
   * @param goal
   * @return vector x, y, z in Translation3d needed for aiming while standing still
   */
  public Translation3d getShotVector() {
    double velocity = getFormulaShooterRPM() * 0.75;
    double shoulderAngle = getFormulaShoulderAngle().getRadians();
    double driveAngle = getFormulaDriveAngle().getRadians();

    double xy = velocity * Math.cos(shoulderAngle); // xy vector of shoulder/swerve 
    double x = xy * Math.cos(driveAngle); // x vector of swerve
    double y = xy * Math.sin(driveAngle); // y vector of swerve
    double z = velocity * Math.sin(shoulderAngle); // z vector/angle of shoulder 
    return new Translation3d(x, y, z);
  }

  /**
   * Get the vector (angle and speed) for shooting into goal while moving
   * @param goal
   * @return vector x, y, z in Translation3d needed for aiming while moving
   */
  public Translation3d getMovingShotVector() {
    Translation2d speed = Swerve.getInstance().getFOSpeeds();
    double xSpeed = speed.getX(); 
    double ySpeed = speed.getY();

    Translation3d stillShotVector = getShotVector();

    //new moving vectors need to get xy speed instead of just angles 
    return new Translation3d(stillShotVector.getX() - xSpeed, stillShotVector.getY() - ySpeed, stillShotVector.getZ());
  }


  /**
   * Get horizontal angle that serve should turn to while aiming and moving 
   * @param goal 
   * @return swerve rotation needed for aiming while moving
   */
  public Rotation2d getMovingDriveAngle() {
    Translation3d movingShotVector = getMovingShotVector();

    //get the radians of the arctan of the slope of y and x of moving vector in radian
    return Rotation2d.fromRadians(Math.atan2(movingShotVector.getY(), movingShotVector.getX()));
  }

  /**
   * Get vertical angle that shoulder should turn to while aiming and moving
   * @param goal
   * @return shoulder rotation
   */
  public Rotation2d getMovingShoulderAngle() {
    Translation3d movingShotVector = getMovingShotVector();

    double xy = Math.atan2(movingShotVector.getY(), movingShotVector.getX());
    //get the radians of the arctan of the slope of vertical and horizontal component
    return Rotation2d.fromRadians(Math.atan2(movingShotVector.getZ(), xy));
  }


  /**
   * get the rpm that the shooter should shoot at based on robot's translation position
   * @param goal
   * @return rpm in double 
   */
  public double getMovingShooterRPM() {
    Translation3d movingShotVector = getMovingShotVector();
    return movingShotVector.getNorm() * 4.0 / 3.0; 
  }

  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  public Alliance getAllianceColor() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() ? alliance.get() : DriverStation.Alliance.Blue;
  }

  public void setGoals() {
    var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        speakerLocation = 
          alliance.get() == DriverStation.Alliance.Blue ? 
          Constants.field.BLUE_SPEAKER_LOCATION :
          Constants.field.RED_SPEAKER_LOCATION;
        cornerLocation = 
          alliance.get() == DriverStation.Alliance.Blue ? 
          Constants.field.BLUE_CORNER_LOCATION :
          Constants.field.RED_CORNER_LOCATION;
      }
  }
}