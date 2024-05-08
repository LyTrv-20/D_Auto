// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 //CONSTANTS SHOULD BE CHANGABLE FOR NEXT GAME SZNS
public final class Constants {
  //Game constraints should change according to da rules 
  public static class GameConstraints{
    public static final int TIME_LIMIT = 15;
    public static final int FOUL = 2;
    public static final int TECH_FOUL = 5;

    public static final int ALLOWED_GAME_ELEMENT = 1;
    public static final int NOTE = -5; //negative bc less =  higher priority
    //TODO: add distance restriction for shooting -> no farther than the wing tape mark in AUTO -> no full court
    //TODO: add priority in avoiding robots AFTER passing the center line 
  }

  public static enum STATE{
    NEW, OPEN, CLOSED
  }

  public static class KnownStaticMapPos {
    public static final Pose2d STAGE_PODIUM_CLOSE = new Pose2d();
    public static final Pose2d STAGE_PODIUM_AMP = new Pose2d();
    public static final Pose2d STAGE_PODIUM_SOURCE = new Pose2d();
    public static final Pose2d WING_LINE = new Pose2d();
    public static final Pose2d MIDLINE = new Pose2d();
    public static final Pose2d SOURCE = new Pose2d();
    public static final Pose2d AMP = new Pose2d();
  }
}
