// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.RobotContainer;
//import frc.robot.subsystems.FeederBase;
//import frc.robot.subsystems.IntakeBase;
//import frc.robot.subsystems.LauncherBase;

public class Launch1to2Ball extends CommandBase {


  public Launch1to2Ball() {
    //this.auto = auto;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Robot.m_robotContainer.m_LauncherBase.setRPM(750);
    Robot.m_robotContainer.m_LauncherBase.setPO(0.15);

    //   //updates the ball position
    // prevBall = currentBall;
    // currentBall = m_debouncer.calculate(!beamBreak.get());

    //   //updates the ballcount
    // if(prevBall != currentBall && currentBall == true){
    //   ballCount++;
    // }

    //IntakeBase.IntakeMotor.set(Constants.IMSpeed);

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    //Robot.m_robotContainer.m_LauncherBase.setRPM(0);
    Robot.m_robotContainer.m_LauncherBase.setPO(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Robot.m_robotContainer.opJoy.getStartButton();
  }
}
