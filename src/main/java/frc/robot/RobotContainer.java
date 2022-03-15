// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

//import java.util.concurrent.TimeUnit;

//import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoLaunch;
import frc.robot.commands.BeamBreakTriggered;
import frc.robot.commands.BottomFeederActivate;
//import frc.robot.commands.BottomFeederReverse;
import frc.robot.commands.IntakeActivate;
//import frc.robot.commands.IntakeReverse;
import frc.robot.commands.Launch1to2Ball;
import frc.robot.commands.SetBlinkin;
import frc.robot.commands.TopFeederActivate;
import frc.robot.subsystems.BlinkinBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.FeederBase;
import frc.robot.subsystems.HangerBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.LauncherBase;
import frc.robot.subsystems.LimeLightBase;
import frc.robot.subsystems.TurretBase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //SUBSYSTEMS---------------------------------------------------------------------------------------------------
  public final DriveBase m_DriveBase = new DriveBase();
  public final FeederBase m_FeederBase = new FeederBase();
  public final IntakeBase m_IntakeBase = new IntakeBase();
  public final LimeLightBase m_LimeLightBase = new LimeLightBase();
  public final LauncherBase m_LauncherBase = new LauncherBase();
  public final BlinkinBase m_BlinkinBase = new BlinkinBase(Constants.blink);
  public final TurretBase m_TurretBase = new TurretBase();
  public final HangerBase m_HangerBase = new HangerBase();

  //COMMANDS-----------------------------------------------------------------------------------------------------
  public final BottomFeederActivate m_BFA = new BottomFeederActivate(false);
  public final TopFeederActivate m_TFA = new TopFeederActivate(false);
  public final Launch1to2Ball m_Launch12 = new Launch1to2Ball(true);
  public final Command m_SetBlink = new SetBlinkin(Constants.strobeGold);
  
  //VARIABLES---------------------------------------------------------------------------------------------------- 
  public boolean  BButtonToggle = false,
                  YButtonToggle = false,
                  prevBall,
                  currentBall;
  public static double  setpoint,
                        blinkPattern = Constants.autoIdle,
                        JOY_DEADZONE = 0.1;
  public static int ballCount = 0;

  //OTHER--------------------------------------------------------------------------------------------------------
  public final SlewRateLimiter filter = new SlewRateLimiter(0.8);
  public DigitalInput beamBreak = new DigitalInput(0);
  Debouncer m_debouncer = new Debouncer(0.06, Debouncer.DebounceType.kBoth);
  public SendableChooser<String> autoChooser = new SendableChooser<String>();
  public Command m_autonomousCommand;
  public Trajectory trajectory;

  //JOYSTICKS ---------------------------------------------------------------------------------------------------
  public final XboxController driveJoy = new XboxController(0),
                              opJoy = new XboxController(1);
  public final JoystickButton aButton = new JoystickButton(opJoy, Constants.kA),
                              bButton = new JoystickButton(opJoy, Constants.kB),
                              yButton = new JoystickButton(opJoy, Constants.kY),
                              xButton = new JoystickButton(opJoy, Constants.kX),
                              startButton = new JoystickButton(opJoy, Constants.kStart);
  
  public POVButton  right = new POVButton(opJoy, 90),
                    downRight = new POVButton(opJoy, 135),
                    down = new POVButton(opJoy, 180),
                    downLeft = new POVButton(opJoy, 225),
                    left = new POVButton(opJoy, 270);

    //returns the joystick values from -1 to 1, implements a deadzone around zero to prevent drift
  public double getDriveJoy(int axis) {
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getOpJoy(int axis) {
    double raw = opJoy.getRawAxis(axis);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getDriveJoyXR() {
    double raw = getDriveJoy(Constants.XR);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getDriveJoyYL() {
    double raw = getDriveJoy(Constants.YL);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  //------------------------------------------------------------------------------------------------------------
  




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  //ROBOT INIT -------------------------------------------------------------------------------------------------

  public void roboInit(){
    ballCount = 0;
    setpoint = 0;
    
    //autonomous paths 
    autoChooser.addOption("Taxi", "Taxi");
    autoChooser.addOption("Taxi 1 Ball", "Taxi 1 Ball");
    autoChooser.addOption("Taxi 2 Ball", "Taxi 2 Ball");
    autoChooser.addOption("Taxi 2 Ball Outbound", "Taxi 2 Ball Outbound");
    autoChooser.addOption("Taxi 3 Ball", "Taxi 3 Ball");
    autoChooser.addOption("Taxi 4 Ball", "Taxi 4 Ball");

    m_DriveBase.resetEncoders();
    SmartDashboard.putData("Auto Routine", autoChooser);

    startButton.whenHeld(launchCommand(true));
    xButton.whenHeld(launchCommand(false));
    //aButton.whenHeld(ejectBottom());
    //bButton.whenHeld(ejectTop());
    m_HangerBase.hangerSol.set(Value.kForward);
  }

  public void roboPeriodic(){
      
    if (m_LauncherBase.rLaunchMotor.get()!=0||opJoy.getStartButton()||opJoy.getXButton()){
      ballCount = 0;
      m_IntakeBase.intakeSol1.set(Value.kReverse);
    } else {

      if (ballCount >= 2 && !opJoy.getAButton()){
        m_BFA.stop();
      } else if (opJoy.getAButton()){
        m_BFA.reverse();
        ballCount--;
      } else if (m_IntakeBase.intakeSol1.get()==Value.kForward){
        m_BFA.stop();
      } else {
        m_BFA.execute();
     }     
    }
    m_TurretBase.setPos(setpoint);

  }

  //AUTO INIT --------------------------------------------------------------------------------------------------

  public void autoInit(){
    //m_IntakeBase.intakeSol1.set(Value.kForward);
    //m_BlinkinBase.set(Constants.autoIdle);
    // setpoint = -10;
    m_DriveBase.resetEncoders();
    m_DriveBase.m_gyro.reset();
    blinkPattern = Constants.autoIdle;
    m_BlinkinBase.set(blinkPattern); //CHANGE SECONDS
    m_IntakeBase.intakeSol1.set(Value.kReverse);
    if (autoChooser.getSelected() != null){
      m_autonomousCommand = getAutonomousCommand(autoChooser.getSelected());
      m_autonomousCommand.schedule();
    }

   

  }

  public void teleopInit(){

    m_IntakeBase.intakeSol1.set(Value.kReverse);
  }

  //TELEOP PERIODIC --------------------------------------------------------------------------------------------

  public void telePeroidic(){

    //m_limeLightBase.periodic(); //updates limelight vars
    prevBall = currentBall;
    currentBall = m_debouncer.calculate(!beamBreak.get());
  
      //updates the ballcount
    if(prevBall != currentBall && currentBall){
        ballCount++;
        //blinkPattern = Constants.green;
    }

    SmartDashboard.putBoolean("Hall Effect", m_TurretBase.getHallEffect());
    SmartDashboard.putNumber("The new ballCount", ballCount);

    //DRIVEBASE
      SmartDashboard.putNumber("Drive Encoder Left", m_DriveBase.leftEncPos);
      SmartDashboard.putNumber("Drive Encoder Right", m_DriveBase.rightEncPos);
     
      SmartDashboard.putNumber("Drive PO Left", m_DriveBase._left1.getMotorOutputPercent());
      SmartDashboard.putNumber("Drive PO Right", m_DriveBase._right1.getMotorOutputPercent());
      SmartDashboard.putNumber("Gyro", m_DriveBase.m_gyro.getYaw());
      SmartDashboard.putNumber("getRPM", m_LauncherBase.getRPM());
      SmartDashboard.putNumber("actual RPM", m_LauncherBase.rLaunchMotor.getSelectedSensorVelocity());
      //Switches between curvature and arcade

      if (driveJoy.getBButtonPressed()){
        BButtonToggle = !BButtonToggle;
      }

      if (BButtonToggle) {
        m_DriveBase.m_drive.curvatureDrive(filter.calculate(-getDriveJoy(Constants.YL)), getDriveJoyXR(), driveJoy.getBButtonPressed());
        SmartDashboard.putString("Drivetype", "curvature");
      } else {
        m_DriveBase.m_drive.arcadeDrive(filter.calculate(-getDriveJoy(Constants.YL)), getDriveJoyXR());
        SmartDashboard.putString("Drivetype", "arcade");
      }
      

    //INTAKE
      if(opJoy.getRightTriggerAxis() > 0.01){
        m_IntakeBase.IntakeMotor.set(Constants.IMSpeed);
      } else if(opJoy.getLeftTriggerAxis() > 0.01){
        m_IntakeBase.IntakeMotor.set(-Constants.IMSpeed);
      } else {
        m_IntakeBase.IntakeMotor.set(0.0);
      }


      // if(m_FeederBase.BottomFeederMotor.get()!=0){
      //   m_IntakeBase.intakeSol1.set(Value.kReverse);
      // }else{
        
      // }
      
      if(opJoy.getBButtonPressed()){
        m_IntakeBase.intakeSol1.toggle();
        //m_IntakeBase.intakeSol2.toggle();
      }
      

      //TURRET     
      m_TurretBase.resetEncoder();

      //if (getOpJoy(Constants.XR) == 0.0){
        if (right.get()){
          setpoint = 90;
        } else if (downRight.get()){
          setpoint = 45;
        } else if (down.get()){
          setpoint = 0;
        } else if (downLeft.get()){
          setpoint = -45;
        } else if (left.get()){
          setpoint = -90;
        }

      setpoint -= getOpJoy(Constants.XR)*10;  
      //m_TurretBase.setPos(setpoint);
    
      if(opJoy.getLeftBumper()){
        //blinkPattern = Constants.strobeGold;
        setpoint = 90;
        m_HangerBase.hangerSol.set(Value.kReverse);
        m_HangerBase.hangerMotors.set(0.9);
      } else if(opJoy.getRightBumper()){
        setpoint = 90;
        //blinkPattern = Constants.green;
        m_HangerBase.hangerMotors.set(-0.9);
      } else{
        m_HangerBase.hangerMotors.set(0);
      }

      if(opJoy.getYButtonPressed()){
        m_HangerBase.hangerSol.toggle(); 
      }
        //UNCOMMENT ONCE SOFT LIMITS SET, DON'T DELETE (JUST YET)
      // if (m_HangerBase.leftHangerMotor.getEncoder().getPosition() > m_HangerBase.leftHangerMotor.getSoftLimit(SoftLimitDirection.kForward)-2){
      //   //blinkPattern = Constants.strobeGold;
      // } else if(m_HangerBase.leftHangerMotor.getEncoder().getPosition() < m_HangerBase.leftHangerMotor.getSoftLimit(SoftLimitDirection.kReverse)+2){
      //   //blinkPattern = Constants.green;
      // }

        //BATTERY TO LED
      // if(RobotController.getBatteryVoltage() < 11.0){
      //   blinkPattern = Constants.strobeGold;
      // } if( RobotController.getBatteryVoltage() < 9){
      //   blinkPattern = Constants.red;
      // }
      if(RobotController.getBatteryVoltage() < 7){
        blinkPattern = Constants.red;
      } else if(opJoy.getLeftBumper()){
        blinkPattern = Constants.strobeGold;
       // setDankLEDs(blinkPattern, 1.0);
      } else if(prevBall != currentBall && currentBall || opJoy.getRightBumper() ){
        blinkPattern = Constants.green;
      } else {
        blinkPattern = Constants.teleOpIdle;
        //m_BlinkinBase.set(blinkPattern);
      }
      m_BlinkinBase.set(blinkPattern);
        //new SetBlinkin(Constants.violet);
    }

  //COMMANDS METHODS --------------------------------------------------------------------------------------------

    public Command launchCommand(boolean isShootingLow){
      // SequentialCommandGroup topFeeder = new SequentialCommandGroup(new WaitCommand(0.1), new TopFeederActivate());
      // SequentialCommandGroup bottomFeeder = new SequentialCommandGroup(new WaitCommand(0.3), new BottomFeederActivate());
      // return new ParallelCommandGroup(m_Launch12, topFeeder, bottomFeeder);
      //m_IntakeBase.intakeSol1.set(Value.kReverse);
      if(isShootingLow){
        return new Launch1to2Ball(isShootingLow)
                 .alongWith(new WaitCommand(0.4)
                 .andThen(new TopFeederActivate(false)))
                 .alongWith(new WaitCommand(0.3)
                 .andThen(new BottomFeederActivate(false)));
      }
        return new Launch1to2Ball(isShootingLow)
                  .alongWith(new WaitCommand(1.2)
                  .andThen(new TopFeederActivate(false)))
                  .alongWith(new WaitCommand(0.9)
                  .andThen(new BottomFeederActivate(false)));
    }

    public Command timedLaunchCommand(double lSeconds, double fSeconds, double bSeconds){
      // SequentialCommandGroup topFeeder = new SequentialCommandGroup(new WaitCommand(0.1), new TopFeederActivate());
      // SequentialCommandGroup bottomFeeder = new SequentialCommandGroup(new WaitCommand(0.3), new BottomFeederActivate());
      // return new ParallelCommandGroup(m_Launch12, topFeeder, bottomFeeder);
      return new ParallelRaceGroup(new AutoLaunch(), new WaitCommand(lSeconds))
                 .alongWith(new WaitCommand(1)
                 .andThen(new ParallelRaceGroup(new TopFeederActivate(true), new WaitCommand(fSeconds))))
                 .alongWith(new WaitCommand(1.5)
                 .andThen(new ParallelRaceGroup(new BottomFeederActivate(true), new WaitCommand(bSeconds))));
    }

    // public Command ejectTop(){
    //   return new Launch1to2Ball()
    //              .alongWith(new WaitCommand(0.2))
    //              .andThen(new TopFeederActivate(false))
    //              .alongWith(new ChangeBallCountBy(1));
    //   //the ball count should have been set to 0 so we add one after since we on
    // }

    // public Command ejectBottom(){
    //   return m_BFA.execute();

    //   // return new BottomFeederReverse()
    //   //            .alongWith(new IntakeReverse())
    //   //           // .alongWith(new ChangeBallCountBy(-1));
    //   //            .alongWith(new ParallelRaceGroup(new ChangeBallCountBy(-1), new WaitCommand(0.02)));
    // }

    public Command setDankLEDs(double pattern, double time){
      return new ParallelRaceGroup(new SetBlinkin(pattern), new WaitCommand(time));
    }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String path) {
    switch(path){
    case "Taxi":
      return pathFollow("output/Taxi.wpilib.json", false);
    case "Taxi 1 Ball":
      return timedLaunchCommand(3, 2, 1.5)
            .andThen(
              pathFollow("output/Taxi.wpilib.json", false));
    case "Taxi 2 Ball":
      setpoint = -10;
      return new IntakeActivate()
            .alongWith(
              new ParallelRaceGroup(
                new BottomFeederActivate(true), 
                new BeamBreakTriggered(1)))
            .alongWith(
              pathFollow("output/Taxi.wpilib.json", false)
              .andThen(
                pathFollow("output/TaxiRev.wpilib.json", true)
              .andThen(
                timedLaunchCommand(4.5, 3.5, 3))));
    case "Taxi 2 Ball Outbound":
      setpoint = 45;
      return new IntakeActivate()
            .alongWith(
              new ParallelRaceGroup(
                new BottomFeederActivate(true), 
                new BeamBreakTriggered(1)))
            .alongWith(
              pathFollow("output/Taxi.wpilib.json", false)
              .andThen(
                pathFollow("output/TaxiRev.wpilib.json", true)
              .andThen(
                timedLaunchCommand(4.5, 3.5, 3))));
    case "Taxi 3 Ball":
      return new IntakeActivate()
            .alongWith(timedLaunchCommand(3, 2, 1.5)
            .andThen(
              pathFollow("output/Taxi3.wpilib.json", false).alongWith(new ParallelRaceGroup(new BeamBreakTriggered(2), new BottomFeederActivate(true))))
            .andThen(
              new WaitCommand(0.1))
            .andThen(
              pathFollow("output/Taxi3Rev.wpilib.json", true))
            .andThen(
              timedLaunchCommand(4.5, 3.5, 3)));
    case "Taxi 4 Ball":
      return //Taxi 2 Ball
            new IntakeActivate()
              .alongWith(
                new ParallelRaceGroup(
                  new BottomFeederActivate(true), 
                  new BeamBreakTriggered(1))
              .alongWith(
                  pathFollow("output/Taxi.wpilib.json", false)
                .andThen(
                  pathFollow("output/TaxiRev.wpilib.json", true)
                .andThen(
                  timedLaunchCommand(4.5, 3.5, 3))))
            //Taxi 2 Ball with Long Path
              .andThen(
                pathFollow("output/Taxi3.wpilib.json", true).alongWith(new ParallelRaceGroup(new BeamBreakTriggered(2), new BottomFeederActivate(true))))
              .andThen(
                new WaitCommand(0.1))
              .andThen(
                pathFollow("output/Taxi3Rev.wpilib.json", true))
              .andThen(
                timedLaunchCommand(4.5, 3.5, 3)));
    }
    return null;
  }

  public Command pathFollow(String trajectoryJSON, boolean multiPath){
    try {
      Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
    } catch (final IOException ex) {
      // TODO Auto-generated catch block
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    //m_drivebase.m_gyro.reset();
    
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
                                                    m_DriveBase::getPose,
                                                    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                                    new SimpleMotorFeedforward(Constants.ksVolts, 
                                                                               Constants.kvVoltSecondsPerMeter,
                                                                               Constants.kaVoltSecondsSquaredPerMeter),
                                                    Constants.m_driveKinematics,
                                                    m_DriveBase::getWheelSpeeds,
                                                    new PIDController(Constants.kP, 0, 0),
                                                    new PIDController(Constants.kP, 0, 0),
                                                    m_DriveBase::voltageControl,
                                                    m_DriveBase);
    
    // Run path following command, then stop at the end.
    // Robot.m_robotContainer.m_driveAuto.m_drive.feed();
    //m_drivebase.resetOdometry(trajectory.getInitialPose());
    if (!multiPath){
      m_DriveBase.resetOdometry(trajectory.getInitialPose());
    } 
    return ramseteCommand;
  }
}
