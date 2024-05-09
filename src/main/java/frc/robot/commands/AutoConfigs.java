package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

/** Add your docs here. */
public class AutoConfigs {
  public static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(Constants.drivetrain.MAX_VELOCITY, Constants.drivetrain.MAX_ACCEL, 1, 1);

  public static final HashMap<String, Command> EVENTS =
      new HashMap<>(
          Map.ofEntries(
            Map.entry("Delay", new WaitCommand(2.0))
      ));
}
