package CSP_Lib.motors;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import CSP_Lib.utils.TempManager;

public class CSP_TalonFX extends TalonFX implements CSP_Motor {

  /**
   * Creates a CSP_Falcon object
   *
   * @param id CAN ID of the Falcon 500
   * @param canBus name of the CAN Bus the Falcon is on
   */
  public CSP_TalonFX(int id, String canBus) {
    super(id, canBus);
    init();
  }

  /**
   * Creates a CSP_Falcon object with assumed Roborio CAN Bus
   *
   * @param id CAN ID of the Falcon 500
   */
  public CSP_TalonFX(int id) {
    super(id, "rio");
    init();
  }

  /** Configures the motor for typical use */
  public void init() {


    setEncoderDegrees(0.0);

    TempManager.addMotor(this);
  }

  /**
   * Sets the motor to a percentage of its power
   *
   * @param percent desired percentage of power with range [-1.0, 1.0]
   */
  public void set(double percent) {
    super.set(percent);
  }

  /**
   * Sets the motor to a voltage
   *
   * @param voltage desired number of volts
   */
  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
  }

  /**
   * Sets whether the motor is inverted
   *
   * @param inverted true: inverted, false: not inverted
   */
  public void setInverted(boolean inverted) {
    super.setInverted(inverted);
  }

  /**
   * Sets whether the motor brakes
   *
   * @param braked true: brake, false: coast
   */
  public void setBrake(boolean braked) {
    super.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /** 
   * Sets the ramp rate of the motor
   *
   * @param rampRate time it takes to ramp up in seconds
   */
  public void setRampRate(double rampRate) {
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = rampRate;
    super.getConfigurator().apply(openLoopRampsConfigs);

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = rampRate;
    super.getConfigurator().apply(closedLoopRampsConfigs);
  }

  /**
   * Sets the encoder to a desired position
   *
   * @param position desired position to set encoder to
   */
  public void setEncoderDegrees(double position) {
    super.getConfigurator().setPosition(position / (360.0));
  }

  /**
   * Returns the position of the motor
   *
   * @return position of the motor
   */
  public double getPositionDegrees() {
    return super.getRotorPosition().getValueAsDouble() * 360.0;
  }

  /**
   * Returns the velocity of the motor  
   *
   * @return velocity of the motor
   */
  public double getRPM() {
    return ((super.getVelocity().getValueAsDouble() * 60.0));
  }

  /**
   * Returns the current of the motor
   *
   * @return current of the motor in amps
   */
  public double getCurrent() {
    return super.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Returns the temperature of the motor
   *
   * @return temperature of the motor in Celcius
   */
  public double getTemperature() {
    return super.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Returns the CAN ID of the motor
   *
   * @return motor CAN ID
   */
  public int getID() {
    return super.getDeviceID();
  } 
}
