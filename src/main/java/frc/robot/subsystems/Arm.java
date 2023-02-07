// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private WPI_TalonFX m_armMotor;
  private CANCoder m_encoder;

  // Encoder angle offset
  private double m_angleOffset;

  // Gear ratio of arm
  private double m_gearRatio;

  private ProfiledPIDController m_pidController;
  private ArmFeedforward m_feedForwardController;
  
  private double kp, ki, kd;

  private double ks, kg, kv, ka;

  private double maxVelocity, maxAcceleration;


  /** Creates a new Arm. */
  public Arm() {
    this.m_armMotor = new WPI_TalonFX(ArmConstants.armMotorID);
    configArmMotor();

    this.m_encoder = new CANCoder(ArmConstants.armEncoderID);
    configArmCanCoder();

    this.m_pidController = new ProfiledPIDController(kp, ki, kd, 
    new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    this.m_feedForwardController = new ArmFeedforward(ks, kg, kv, ka);

    this.m_angleOffset = ArmConstants.angleOffset;

    this.m_gearRatio = ArmConstants.gearRatio;
    
    this.maxVelocity = ArmConstants.velConstraint;
    this.maxAcceleration = ArmConstants.accelConstraint;
    
  }

  public void configArmMotor() {
    m_armMotor.configFactoryDefault();
    m_armMotor.configAllSettings(Robot.ctreConfigs.armMotorConfig);
    m_armMotor.setInverted(ArmConstants.motorInvert);
  }

  public void configArmCanCoder() {
    m_encoder.configFactoryDefault();
    m_encoder.configAllSettings(Robot.ctreConfigs.armCanCoderConfig);
  }

  public double getAbsoluteAngle() {
    return Conversions.CANcoderToDegrees(m_encoder.getAbsolutePosition(), m_gearRatio);
  }

  public double getAngle() {
    return getAbsoluteAngle()-m_angleOffset;
  }

  public void setGoal(double angle) {
    m_pidController.setGoal(angle);
  }

  public double getGoal() {
    return m_pidController.getGoal().position;
  }

  @Override
  public void periodic() {

    m_armMotor.setVoltage(
      /** PID Controller calculates output based on 
      the current position and the goal **/
      m_pidController.calculate(getAngle()) 
      /** Feedforward uses setpoints calculated by 
      motion profiling **/
    + m_feedForwardController.calculate(
      m_pidController.getSetpoint().position, 
    m_pidController.getSetpoint().velocity));
  }
}
