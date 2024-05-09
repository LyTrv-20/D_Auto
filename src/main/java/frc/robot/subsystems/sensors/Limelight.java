// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import CSP_Lib.utils.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Limelight {
  private String name;
  private Pose3d pose = new Pose3d();
  private Pose2d currentPose = new Pose2d();


  public Limelight(String name, Pose3d pose) {
    this.name = name;
    this.pose = pose;
    init();
  }

  private void init() {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        Units.radiansToDegrees(pose.getRotation().getX()),
        Units.radiansToDegrees(pose.getRotation().getY()),
        Units.radiansToDegrees(pose.getRotation().getZ()));
  }

  public Pose2d getPose2d() {
    if (LimelightHelpers.getTV(name)) {
      currentPose = LimelightHelpers.getBotPose3d_wpiBlue(name).toPose2d();
      return currentPose;
    } else {
      return new Pose2d();
    }
  }
  
  public boolean getTV() {
    return LimelightHelpers.getTV(name);
  }

  public double getLatency() {
    double time = Timer.getFPGATimestamp();
    if (LimelightHelpers.getTV(name)) {
      return time
          - (LimelightHelpers.getLatency_Capture(name) / 1000)
          - (LimelightHelpers.getLatency_Pipeline(name) / 1000);
    } else return 0.0;
  }

  public double getAvgTagDistance() {
    return getTV() ? LimelightHelpers.getBotPose(name)[9] : 0;
  }
}
