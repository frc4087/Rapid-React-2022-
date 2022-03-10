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


public class AutoLaunch extends CommandBase {

    //private final LauncherBase m_LauncherBase;
    //private int time;
    //private double seconds;

    public AutoLaunch(/*LauncherBase launcherBase, double seconds*/) {
        //m_LauncherBase = launcherBase;
        //this.seconds = seconds;
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(launcherBase);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //time = 0;
        //m_LauncherBase.setRPM(750);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //time++;
        // Robot.m_robotContainer.m_LauncherBase.setRPM(1000);
        Robot.m_robotContainer.m_LauncherBase.setPO(0.3);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        // Robot.m_robotContainer.m_LauncherBase.setRPM(0);
        Robot.m_robotContainer.m_LauncherBase.setPO(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        /*turns the seconds into units of 20ms 
         *because thats how many times execute should have run
         */
        return false;//time>=seconds*1000/20;
    }
}
