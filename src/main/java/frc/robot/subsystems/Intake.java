// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {

  private CANSparkMax m_motor;
  private double speed;

  /** Creates a new Intake. */
  public Intake() {
    this.m_motor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
    configure();

    this.speed = IntakeConstants.speed * -1;
  }
  public void configure() {
    m_motor.setInverted(IntakeConstants.inverted);
    m_motor.restoreFactoryDefaults();
  }

  public void intake() {
    m_motor.set(speed);
  }

  public void outtake() {
    m_motor.set(-speed);
  }

  public void stop() {
    m_motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
