// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private Arm s_Arm;
  private Intake s_Intake;
  private boolean isIntaking;
  private double seconds;
  private double t_s;


  public IntakeCommand(Arm arm, Intake intake, double d, boolean intaking) {
    s_Arm = arm;
    s_Intake = intake;
    isIntaking = intaking;
    seconds = d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    t_s = System.currentTimeMillis()/1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isIntaking) {
      if (s_Arm.getSetPoint()>5 && s_Arm.getSetPoint() < 60){ // Hardcoded limits for shelf intake
        s_Intake.intake();// Start intaking
        s_Arm.setGoal(s_Arm.getSetPoint()-5); // Move intake downwards (waits for arm to reach goal)
      }
      else if(s_Arm.getSetPoint()>180 && s_Arm.getSetPoint()< 220){ // Hardcoded limits for low intake
        s_Intake.intake(); // Start intaking
        s_Arm.setGoal(s_Arm.getSetPoint()+20); // Move intake downwards (waits for arm to reach goal
      }
      else{
        s_Intake.intake();
      } // Base case where current goal is outside of front intaking limits
      

    }

    else {
      if (s_Arm.getSetPoint()>160 && s_Arm.getSetPoint() < 235){ // Hardcoded limits for outtake
      
        s_Intake.outtake(); // Start intaking
        s_Arm.setGoal(s_Arm.getSetPoint()-5);// Move intake downwards (waits for arm to reach goal)
      }
      else{ // Only use base case for outtaking
        s_Intake.outtake();
      }
        
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stop();
    if(isIntaking){
      if(s_Arm.getSetPoint()>170 && s_Arm.getSetPoint()< 240){
        s_Arm.setGoal(s_Arm.getSetPoint()-15);
      }
      else if(s_Arm.getSetPoint()>-10 && s_Arm.getSetPoint() < 60){
        s_Arm.setGoal(s_Arm.getSetPoint()+15);
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis()/1000.0-t_s > seconds);
  }
}
