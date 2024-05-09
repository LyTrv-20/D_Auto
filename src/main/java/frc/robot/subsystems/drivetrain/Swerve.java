// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import CSP_Lib.utils.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;

public class Swerve extends SubsystemBase {
  private static Swerve instance = null;

  /**
   * Singleton Constructor for {@link Swerve}
   * @return Single instance of {@link Swerve} common to all contexts.
   */
  public static synchronized Swerve getInstance() {
    if (instance == null) instance = new Swerve();
    return instance;
  }

  private static Translation2d[] getLocations(SwerveModule... modules) {
    Translation2d[] locations = new Translation2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      locations[i] = modules[i].getLocation();
    }
    return locations;
  }

  

  private SwerveModule[] moduleList = {
    new SwerveModule(
      "Front Left",
      Constants.drivetrain.FL_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.FL_SPEED,
      Constants.ids.FL_ANGLE,
      Constants.ids.FL_ENCODER,
      Constants.drivetrain.FL_ZERO
    ),
    new SwerveModule(
      "Front Right",
      Constants.drivetrain.FR_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.FR_SPEED,
      Constants.ids.FR_ANGLE,
      Constants.ids.FR_ENCODER,
      Constants.drivetrain.FR_ZERO
    ),
    new SwerveModule(
      "Back Left",
      Constants.drivetrain.BL_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.BL_SPEED,
      Constants.ids.BL_ANGLE,
      Constants.ids.BL_ENCODER,
      Constants.drivetrain.BL_ZERO
    ),
    new SwerveModule(
      "Back Right",
      Constants.drivetrain.BR_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.BR_SPEED,
      Constants.ids.BR_ANGLE,
      Constants.ids.BR_ENCODER,
      Constants.drivetrain.BR_ZERO
    )
  };

  private SwerveModulePosition[] getSwerveModulePositions(SwerveModule... modules) {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getModulePosition();
    }
    return modulePositions;
  }

  private Sensors sensors = Sensors.getInstance();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getLocations(moduleList));

  public double setOmega = 0.0;


  // private Notifier notifier = new Notifier(() -> {
    
  // });

  private Field2d field = new Field2d();

  private SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(
          kinematics,
          sensors.getRotation2d(),
          getSwerveModulePositions(moduleList),
          new Pose2d()
          ,Constants.drivetrain.STATE_STD_DEVS, 
          Constants.drivetrain.VISION_STD_DEVS
          );

  public PIDController rotPID = Constants.drivetrain.ROT_PID;
  public PIDController correctionPID = Constants.drivetrain.CORRECTION_PID;


  /** Creates a new Swerve. */
  private Swerve() {

   // moduleList[1].setAngleSupplyCurrent();

    configurePathPlanner();

    correctionPID.enableContinuousInput(0, 180);
    correctionPID.setTolerance(1.0);

    rotPID.enableContinuousInput(-180.0, 180.0);

    // notifier.startPeriodic(0.2);

    // SmartDashboard.putNumber("Angle kP", Constants.drivetrain.ANGLE_PID.getP());
    // SmartDashboard.putNumber("Angle kD", Constants.drivetrain.ANGLE_PID.getD());

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    updateDashboard();

  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds correctedSpeeds = speeds;

    if (speeds.omegaRadiansPerSecond != 0.0) {
      correctionPID.setSetpoint(sensors.getRotation2d().getDegrees());
    } else if (speeds.vxMetersPerSecond != 0 || speeds.vyMetersPerSecond != 0) {
      double correction = correctionPID.calculate(sensors.getRotation2d().getDegrees());
      correctedSpeeds.omegaRadiansPerSecond = correctionPID.atSetpoint() ? 0.0 : correction;  
    }
    setOmega = correctedSpeeds.omegaRadiansPerSecond;

    setModuleStates(kinematics.toSwerveModuleStates(correctedSpeeds));
  }

  public void setRotationSpeed(double speed) {
    setChassisSpeeds(new ChassisSpeeds(0, 0, speed));
  }

  public void disable() {
    for (SwerveModule module : moduleList) {
      module.zeroPower();
    }
  }

  public void setModuleStates(SwerveModuleState... moduleStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.drivetrain.MAX_VELOCITY);

    if (moduleStates.length == moduleList.length) {
      for (int i = 0; i < moduleList.length; i++) {
          moduleList[i].setModuleState(moduleStates[i]);
      }
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public Translation2d getFOSpeeds() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(getPose2d().getRotation());
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = moduleList[i].getModuleState();
    }
    
    return moduleStates;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose2d() {
    return odometry.getEstimatedPosition();

    
  }

  public Rotation2d getColorNormRotation() {
    if (Sensors.getInstance().getAllianceColor() == DriverStation.Alliance.Blue) {
      return getPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180.0));
    } else {
      return getPose2d().getRotation();
    } 
  }

  public double getSpeed() {
    ChassisSpeeds speeds = getChassisSpeeds();

    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public void updateOdometry() {
    LimelightHelpers.SetRobotOrientation("limelight-back", odometry.getEstimatedPosition().getRotation().getDegrees(), Sensors.getInstance().getPigeonRate(), 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
      if (Math.abs(Sensors.getInstance().getPigeonRate()) <= 720.0 && mt2.tagCount != 0) {
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
  
    LimelightHelpers.SetRobotOrientation("limelight-front", odometry.getEstimatedPosition().getRotation().getDegrees(), Sensors.getInstance().getPigeonRate(), 0, 0, 0, 0);
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
      if (Math.abs(Sensors.getInstance().getPigeonRate()) <= 720.0 && mt2.tagCount != 0) {
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }

    odometry.update(
        sensors.getRotation2d(),
        getSwerveModulePositions(moduleList));
  }

  public void resetOdometry(Pose2d initPose) {
    sensors.resetPigeon(initPose.getRotation());
    odometry.resetPosition(
        sensors.getRotation2d(),
        getSwerveModulePositions(moduleList),
        initPose);

    sensors.resetPigeon(initPose.getRotation());
    odometry.resetPosition(
        sensors.getRotation2d(),
        getSwerveModulePositions(moduleList),
        initPose);

    sensors.resetPigeon(initPose.getRotation());
    odometry.resetPosition(
        sensors.getRotation2d(),
        getSwerveModulePositions(moduleList),
        initPose);

  }

  public void resetPPOdometry(Pose2d initPose) {
    sensors.resetPigeon();
    odometry.resetPosition(
        sensors.getRotation2d(),
        getSwerveModulePositions(moduleList),
        initPose);
  }

  public boolean atGoalAngle(Rotation2d angle) {
    return (Math.abs(getPose2d().getRotation().getDegrees() - angle.getDegrees()) < Math.toDegrees(Math.atan(1.0 / (2.0 * Sensors.getInstance().getSpeakerDistance()))));
  }

  public void updateDashboard() {

    SmartDashboard.putString("Estimated Pose", getPose2d().toString());
    field.setRobotPose(getPose2d());
    // SmartDashboard.putNumber("BackRight Rotations", moduleList[3].getPositionDegrees());
    // SmartDashboard.putNumber("BackLeft Rotations", moduleList[2].getPositionDegrees());
    // SmartDashboard.putNumber("FrontRight Rotations", moduleList[1].getPositionDegrees());
    // SmartDashboard.putNumber("FrontLeft Rotations", moduleList[0].getPositionDegrees());


    SmartDashboard.putNumber("Front Right Angle Temp", moduleList[1].getAngleTemp());
        SmartDashboard.putNumber("Front Left Angle Temp", moduleList[0].getAngleTemp());
    SmartDashboard.putNumber("Back Right Angle Temp", moduleList[3].getAngleTemp());
    SmartDashboard.putNumber("Back Left Angle Temp", moduleList[2].getAngleTemp());

        //SmartDashboard.putNumber("Front Right Angle Volts", moduleList[1].getAngleVolts());

    for (SwerveModule module : moduleList) {
      SmartDashboard.putNumber(module.getName() + " Angle", module.getAngleDegrees());
      
    }

  }

  public void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
            this::getPose2d, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.1,0,0),// Translation PID constants
                    new PIDConstants(2.5,0,0), // Rotation PID constants
                    Constants.drivetrain.MAX_VELOCITY, // Max module speed, in m/s
                    Constants.robot.A_CROSSLENGTH / 2, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field. 
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;

            },
            this // Reference to this subsystem to set requirements
    );
  }
}