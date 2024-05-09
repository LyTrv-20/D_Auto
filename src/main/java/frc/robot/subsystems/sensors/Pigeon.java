package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import CSP_Lib.utils.Conversions;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends Pigeon2 {
  double offsetDegrees;

  public Pigeon(int canID, double offsetDegrees) {
    super(canID, "rio");
    super.getConfigurator().apply(new Pigeon2Configuration());
    super.clearStickyFaults();
    this.offsetDegrees = offsetDegrees;
    reset();
  }

  public void reset() {
    super.setYaw(offsetDegrees + 360.0); // fix formula thing
  }

  public void reset(Rotation2d rotation) {
    super.setYaw(offsetDegrees + rotation.getDegrees() + 360.0);
  }

  public double getRotation() {
    return Conversions.degreesUnsignedToSigned(super.getYaw().getValueAsDouble() % 360.0);
  }

  public double getRate() {
    return super.getRate();
  }

  public double getRollAsDouble() {
    return super.getRoll().getValueAsDouble();
  }

  public double getPitchAsDouble() {
    return super.getPitch().getValueAsDouble();
  }


}
