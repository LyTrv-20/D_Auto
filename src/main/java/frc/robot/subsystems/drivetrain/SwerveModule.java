package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import CSP_Lib.motors.CSP_CANcoder;
import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

  private final String MODULE_NAME;
  private final Translation2d LOCATION;
  private final double GEAR_RATIO;
  private final int SPEED_ID;
  private final int ANGLE_ID;
  private final int ENCODER_ID;
  private final double ZERO;

  private final CSP_TalonFX speed;
  private final CSP_TalonFX angle;
  private final CSP_CANcoder encoder;

  private final PIDController anglePID;
  private final SimpleMotorFeedforward angleFF;
  private final PIDController speedPID;
  private final SimpleMotorFeedforward speedFF;

  public SwerveModule(String moduleName, Translation2d location, double gearRatio, int speedID, int angleID, int encoderID, double zero) {
    this.MODULE_NAME = moduleName;
    this.LOCATION = location;
    this.GEAR_RATIO = gearRatio;
    this.SPEED_ID = speedID;
    this.ANGLE_ID = angleID;
    this.ENCODER_ID = encoderID;
    this.ZERO = zero;

    this.anglePID = Constants.drivetrain.ANGLE_PID;
    this.angleFF = Constants.drivetrain.ANGLE_FF;
    this.speedPID = Constants.drivetrain.SPEED_PID;
    
    this.speedFF = new SimpleMotorFeedforward(Constants.drivetrain.SPEED_FF.ks, Constants.drivetrain.SPEED_FF.kv);

    this.speed = new CSP_TalonFX(SPEED_ID, "rio");
    this.angle = new CSP_TalonFX(ANGLE_ID, "rio");
    this.encoder = new CSP_CANcoder(ENCODER_ID, "rio");

    // TempManager.addMotor(this.speed);
    // TempManager.addMotor(this.angle);

    init();
  }

  public void init() {

    speed.setBrake(false);
    speed.setRampRate(Constants.drivetrain.RAMP_RATE);

    angle.setBrake(false);
    angle.setInverted(true);

    encoder.clearStickyFaults();
    MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
    sensorConfigs.MagnetOffset = -(ZERO / 360.0);
    sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(sensorConfigs);

    anglePID.enableContinuousInput(-180, 180);
    anglePID.setTolerance(0);    
    angle.setEncoderDegrees(getAngleDegrees());


    speed.getVelocity().setUpdateFrequency(50.0);
    speed.getRotorPosition().setUpdateFrequency(50.0);
    speed.getDeviceTemp().setUpdateFrequency(4.0);

    angle.getVelocity().setUpdateFrequency(50.0);
    angle.getRotorPosition().setUpdateFrequency(50.0);
    angle.getDeviceTemp().setUpdateFrequency(4.0);


    encoder.getAbsolutePosition().setUpdateFrequency(100.0);

    speed.optimizeBusUtilization();
    angle.optimizeBusUtilization();
    encoder.optimizeBusUtilization();

    speed.getConfigurator().apply(new CurrentLimitsConfigs()
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(250.0)
    .withSupplyCurrentLimit(50.0));
    speed.clearStickyFaults();
    
        angle.getConfigurator().apply(new CurrentLimitsConfigs()
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(100.0)
    .withSupplyCurrentLimit(50.0));
    angle.clearStickyFaults();
  }

  public void setModuleState(SwerveModuleState desired) {
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngleDegrees()));

    double velocity = optimized.speedMetersPerSecond / Constants.drivetrain.DRIVE_METERS_PER_TICK;
    // pseudocode : setVolts(PID + FF)


      speed.setVoltage(speedPID.calculate(getVelocity(), velocity) + speedFF.calculate(velocity));
    // angle.setVoltage(angleFF.calculate(anglePID.calculate(getAngleDegrees(), optimized.angle.getDegrees())));
      angle.setVoltage(anglePID.calculate(getAngleDegrees(), optimized.angle.getDegrees()));
  }

  /** Sets the speed and angle motors to zero power */
  public void zeroPower() {
    angle.setVoltage(0.0);
    speed.setVoltage(0.0);
  }

  /**
   * Gives the velocity and angle of the module
   *
   * @return SwerveModuleState object containing velocity and angle
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAngleDegrees()));
  }

  /**
   * Gives the position and angle of the module
   *
   * @return SwerveModulePosition object containing position and angle
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getPositionMeters(), Rotation2d.fromDegrees(getAngleDegrees()));
  }

  /**
   * Gives the velocity of the angle
   * 
   * @return double with velocity in Meters Per Second
   */
  private double getVelocity() { // RPM to MPS
    return ((speed.getRPM() / 60.0) * Constants.drivetrain.WHEEL_CIRCUMFRENCE) / GEAR_RATIO;
  }

  /**
   * Gives the angle of the module and constantly corrects it with Cancoder reading
   *
   * @return the angle with range [-180 to 180]
   */
  public double getAngleDegrees() {
    return encoder.getPositionDegrees();
  }

  public double getAnglePIDSetpoint() {
    return anglePID.getSetpoint();
  }

  private double getPositionMeters() {
    return (speed.getPositionDegrees() * Constants.drivetrain.WHEEL_CIRCUMFRENCE) / (360.0 * GEAR_RATIO);
  }

  public String getName() {
    return this.MODULE_NAME;
  }

  public Translation2d getLocation() {
    return this.LOCATION;
  }

  public void setAnglePIDConstants(double kP, double kI, double kD) {
    anglePID.setPID(kP, kI, kD);
  }

  public void setSpeedPIDConstants(double kP, double kI,double kD) {
    speedPID.setPID(kP, kI, kD);
  }

  public double getPositionDegrees() {
    return speed.getPositionDegrees();
  }

  public double getAngleTemp() {
    return angle.getTemperature();
  }

  public double getAngleVolts() {
    return angle.getMotorVoltage().getValueAsDouble();
  }

}
