// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class BottomFeederActivate extends CommandBase {
  private boolean isAuto;
  /** Creates a new BottomFeederActivate. */
  public BottomFeederActivate(boolean auto) {
    isAuto = auto;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.m_FeederBase.BottomFeederMotor.set(Constants.BFMSpeed);
  }

  public void stop(){
    Robot.m_robotContainer.m_FeederBase.BottomFeederMotor.set(0);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_FeederBase.BottomFeederMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAuto){
      return false;
    } else {
      return RobotContainer.ballCount>=1 && !Robot.m_robotContainer.opJoy.getStartButtonPressed();
  
    }
    }
}
