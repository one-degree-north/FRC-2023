// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private WPI_TalonFX m_armMotor;
  private WPI_TalonFX m_armSlave;
  private CANCoder m_encoder;

  // Encoder angle offset
  private double m_angleOffset;

  // Gear ratio of arm

  private ProfiledPIDController m_pidController;
  private ArmFeedforward m_feedForwardController;
  
  private double kp, ki, kd;

  private double ks, kg, kv, ka;

  private double maxVelocity, maxAcceleration;

  public double CANcoderInitTime = 0.0;
  
  private final double LOWLIMIT = -90;

  private final double UPLIMIT = 270;

  private final double TOLERANCE = 5;

  /** Creates a new Arm. */
  public Arm() {
    this.m_armMotor = new WPI_TalonFX(ArmConstants.armMotorID);
    this.m_armSlave = new WPI_TalonFX(ArmConstants.armSlaveID);
    this.m_encoder = new CANCoder(ArmConstants.armEncoderID);

    this.kp = ArmConstants.kP;
    this.ki = ArmConstants.kI;
    this.kd = ArmConstants.kD;

    this.ks = ArmConstants.kS;
    this.kg = ArmConstants.kG;
    this.kv = ArmConstants.kV;
    this.ka = ArmConstants.kA;

    this.maxVelocity = ArmConstants.velConstraint; // Degrees per second
    this.maxAcceleration = ArmConstants.accelConstraint; // Degrees per second squared

    this.m_pidController = new ProfiledPIDController(kp, ki, kd, 
    new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    this.m_feedForwardController = new ArmFeedforward(ks, kg, kv, ka);

    this.m_angleOffset = ArmConstants.angleOffset; // Degrees

    configArmMotor();
    configArmCanCoder();
    resetToAbsolute();
    m_armSlave.follow(m_armMotor);
    m_pidController.disableContinuousInput();

    m_pidController.setTolerance(TOLERANCE);

    setCurrentPosToGoal();
  }

  public void configArmMotor() {
    m_armMotor.configFactoryDefault();
    m_armMotor.configAllSettings(Robot.ctreConfigs.armMotorConfig);
    m_armMotor.setInverted(ArmConstants.motorInvert);

    m_armSlave.configFactoryDefault();
    m_armSlave.configAllSettings(Robot.ctreConfigs.armMotorConfig);
    m_armSlave.setInverted(ArmConstants.slaveInvert);

    m_armSlave.follow(m_armMotor);
  }

  public void configArmCanCoder() {
    m_encoder.configFactoryDefault();
    m_encoder.configAllSettings(Robot.ctreConfigs.armCanCoderConfig);
    resetToAbsolute();
  }

  private void waitForCanCoder(){
    /*
     * Wait for up to 1000 ms for a good CANcoder signal.
     *
     * This prevents a race condition during program startup
     * where we try to synchronize the Falcon encoder to the
     * CANcoder before we have received any position signal
     * from the CANcoder.
     */
    for (int i = 0; i < 100; ++i) {
        m_encoder.getAbsolutePosition();
        if (m_encoder.getLastError() == ErrorCode.OK) {
            break;
        }
        Timer.delay(0.010);            
        CANcoderInitTime += 10;
    }
}

  public void resetToAbsolute(){
    waitForCanCoder();

    double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset, ArmConstants.gearRatio);
    m_armMotor.setSelectedSensorPosition(absolutePosition);
    m_armSlave.setSelectedSensorPosition(absolutePosition);
  }

  public Rotation2d getCanCoder(){
    return Rotation2d.fromDegrees(m_encoder.getAbsolutePosition());
  }

  public double getPosition() {
    return Conversions.falconToDegrees(m_armMotor.getSelectedSensorPosition(), ArmConstants.gearRatio);
  }

  public void setGoal(double angle) {
    if (angle >= LOWLIMIT && angle <= UPLIMIT && Math.abs(getPosition() - angle) <= Math.abs(UPLIMIT-LOWLIMIT)) 
    m_pidController.setGoal(angle);
  }

  public void setCurrentPosToGoal() {
    m_pidController.setGoal(getPosition());
  }

  public double getSetPoint(){
    return m_pidController.getSetpoint().position;
  }

  public double getGoal() {
    return m_pidController.getGoal().position;
  }

  public boolean atGoal() {
    return m_pidController.atGoal();
  }

  public void setPercent(double value) {
    m_armMotor.set(ControlMode.PercentOutput, value);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm CanCoder", getPosition());
    SmartDashboard.putNumber("Arm Right", Conversions.falconToDegrees(m_armMotor.getSelectedSensorPosition(), ArmConstants.gearRatio));
    SmartDashboard.putNumber("Arm Left", Conversions.falconToDegrees(m_armSlave.getSelectedSensorPosition(), ArmConstants.gearRatio));
    // SmartDashboard.putNumber("Current Arm Goal", getGoal());
    SmartDashboard.putNumber("Current Arm Setpoint", m_pidController.getSetpoint().position);
    



    m_armMotor.setVoltage(
      /** PID Controller calculates output based on 
      the current position and the goal **/
      m_pidController.calculate(getPosition())
      /** Feedforward uses setpoints calculated by 
      motion profiling **/
    +
     m_feedForwardController.calculate(
      m_pidController.getSetpoint().position, 
    m_pidController.getSetpoint().velocity)
    );

    // m_armMotor.setVoltage(
    //   /** PID Controller calculates output based on 
    //   the current position and the goal **/
    //   m_pidController.calculate(getPosition()) 
    //   /** Feedforward uses setpoints calculated by 
    //   motion profiling **/
    // + m_feedForwardController.calculate(
    //   m_pidController.getGoal().position, m_pidController.getGoal().velocity));
  }

  

}
