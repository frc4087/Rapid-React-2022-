// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TopFeederActivate extends CommandBase {
  /** Creates a new TopFeederActivate. */
  private boolean isAuto;
  
  public TopFeederActivate(Boolean auto) {
    isAuto = auto;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    Robot.m_robotContainer.m_FeederBase.TopFeederMotor.set(Constants.TFMSpeed);
   
  }

  public void stop(){
    Robot.m_robotContainer.m_FeederBase.TopFeederMotor.set(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_FeederBase.TopFeederMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAuto){
      return false;
    } else {
      return !Robot.m_robotContainer.opJoy.getStartButton();
    }
    
  }
}
