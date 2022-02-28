// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.concurrent.TimeUnit;

//import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.BottomFeederActivate;
import frc.robot.commands.Launch1to2Ball;
import frc.robot.commands.TopFeederActivate;
import frc.robot.commands.Tracking;
import frc.robot.subsystems.BlinkinBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.FeederBase;
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
  // The robot's subsystems and commands are defined here...
  
   // Initialize joysticks
  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);
  public final JoystickButton aButton = new JoystickButton(opJoy, Constants.kA);

  public POVButton right = new POVButton(opJoy, 90);
  public POVButton downRight = new POVButton(opJoy, 135);
  public POVButton down = new POVButton(opJoy, 180);
  public POVButton downLeft = new POVButton(opJoy, 225);
  public POVButton left = new POVButton(opJoy, 270);

  //SUBSYSTEMS
  public final DriveBase m_DriveBase = new DriveBase();
  public final FeederBase m_FeederBase = new FeederBase();
  public final IntakeBase m_IntakeBase = new IntakeBase();
  public final LimeLightBase m_LimeLightBase = new LimeLightBase();
  public final LauncherBase m_LauncherBase = new LauncherBase();
  public final BlinkinBase m_BlinkinBase = new BlinkinBase(Constants.blink);
  public final TurretBase m_TurretBase = new TurretBase();
  public final SlewRateLimiter filter = new SlewRateLimiter(1.0);

  //COMMANDS
  public final BottomFeederActivate m_BFA = new BottomFeederActivate();
  public final TopFeederActivate m_TFA = new TopFeederActivate();
  public final Launch1to2Ball m_Launch12 = new Launch1to2Ball();
  
  //VARIABLES
  public double JOY_DEADZONE = 0.1;
  public boolean BButtonToggle = false;
  public static double setpoint;
  DigitalInput beamBreak = new DigitalInput(0);
  Debouncer m_debouncer = new Debouncer(0.06, Debouncer.DebounceType.kBoth);
  public static int ballCount = 0;
  public boolean prevBall,
                 currentBall;


  // Joystick Methods
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
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  }

  public void roboInit(){
    aButton.whenHeld(launchCommand());
  }

  public void telePeroidic(){
      //m_limeLightBase.periodic(); //updates limelight vars


    prevBall = currentBall;
    currentBall = m_debouncer.calculate(!beamBreak.get());
  
        //updates the ballcount
    if(prevBall != currentBall && currentBall){
        ballCount++;
        BlinkinBase.m_blinkin.set(0.85);
        //wait(0.25);
    }

    //BlinkinBase.m_blinkin.set(0.45);

    SmartDashboard.putBoolean("Hall Effect", m_TurretBase.getHallEffect());
    SmartDashboard.putNumber("The new ballCount", ballCount);

    //DRIVEBASE
      SmartDashboard.putNumber("Drive Encoder Left", m_DriveBase._left1.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Drive Encoder Right", m_DriveBase._right1.getSelectedSensorVelocity());
     
      SmartDashboard.putNumber("Drive PO Left", m_DriveBase._left1.getMotorOutputPercent());
      SmartDashboard.putNumber("Drive PO Right", m_DriveBase._right1.getMotorOutputPercent());
      
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
        IntakeBase.IntakeMotor.set(Constants.IMSpeed);
      } else if(opJoy.getLeftTriggerAxis() > 0.01){
        IntakeBase.IntakeMotor.set(-Constants.IMSpeed);
      } else {
        IntakeBase.IntakeMotor.set(0.0);
      }

      if(opJoy.getBButtonPressed()){
        m_IntakeBase.intakeSol1.toggle();
        //m_IntakeBase.intakeSol2.toggle();
      }

      if (opJoy.getAButton()){
        ballCount = 0;
      } else {
        if (ballCount >= 2){
          m_BFA.stop();
        } else {
          m_BFA.execute();
       }
      }

      //TURRET
     
      m_TurretBase.resetEncoder();

      if (getOpJoy(Constants.XR) == 0.0){
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

        m_TurretBase.setPos(setpoint);
      } else {
        m_TurretBase.turretMotor.set(getOpJoy(Constants.XR));
      }
      
     

    //////////////////////////BEWARE THE SEA OF COMMENTS////////////////////////////

    // //BOTTOM FEEDER
    //   if(opJoy.getRightBumper()){
    //     m_FeederBase.BottomFeederMotor.set(Constants.BFMSpeed);
    //   } else if(opJoy.getLeftBumper()){
    //     m_FeederBase.BottomFeederMotor.set(-Constants.BFMSpeed);
    //   } else {
    //     m_FeederBase.BottomFeederMotor.set(0.0);
    //   }
      
    // //TOP FEEDER
    //    if(opJoy.getYButton()){
    //     m_FeederBase.TopFeederMotor.set(Constants.TFMSpeed);
    //   } else if(opJoy.getAButton()){
    //     m_FeederBase.TopFeederMotor.set(-Constants.TFMSpeed);
    //   } else {
    //     m_FeederBase.TopFeederMotor.set(0.0);
    //   }

    // //LAUNCHER
    //   if(opJoy.getBButton()){
    //     m_LauncherBase.setRPM(520); 
    //   } else{
    //     m_LauncherBase.setRPM(0);
    //   }

    // //TURRET
    //   setpoint = Tracking.turret.turretMotor.getEncoder().getPosition();
      
    // //LAUNCH COMMAND

    // if(opJoy.getAButtonPressed()){
    //   m_Launch12.execute();

    //   if (true){//Robot.m_robotContainer.m_LauncherBase.getRPM() > 700){
    //     m_TFA.execute();
    //     ballCount = 0;
    //     new WaitCommand(1);
    //     m_BFA.execute();
        
    //   }

    // }else if(ballCount < 1){
    //   m_BFA.execute();
    // }else if(ballCount >= 1){
    //   m_BFA.end(true);
    //   //FeederBase.BottomFeederMotor.set(0);
    // } else {
    //   m_Launch12.end(true);
    //   m_TFA.end(true);
    //   m_BFA.end(true);
    // }

  //   if (opJoy.getAButtonPressed()){
  //     launchCommand().schedule();
  //     ballCount = 0;
  //   //   double startTime1 = System.currentTimeMillis(); 
  //   //   m_Launch12.start();

  //   //   if (System.currentTimeMillis() >=  startTime1 + 1000){

  //   //     double startTime2 = System.currentTimeMillis(); 
  //   //     m_TFA.execute();

  //   //     if (System.currentTimeMillis() >=  startTime2 + 1000){

  //   //       m_BFA.execute();
  //   //       ballCount = 0;
  //   //     }
  //   //  }
  //   } else if (!opJoy.getAButtonPressed()){
  //     //m_Launch12.stop();
  //    // m_TFA.stop();
  //     launchCommand().cancel();
  //    //launchCommand().

  //     if (ballCount >= 1){
  //       m_BFA.stop();
  //       //m_BFA.cancel();
  //     } else {
  //       m_BFA.execute();
  //     }

  //  }

       // might have to add a negative
      
        // The robot's subsystems and commands are defined here...
        // public double setpoint = 0;
      
      // if(driveJoy.getXButton()){
  
      //   //turns the robot right if tx is positive
      //   if(m_limeLightBase.getTx()>3){
      //     m_DriveBase.m_drive.arcadeDrive(0,0.3);
      //     SmartDashboard.putString("Direction", "right");
      //   } 
  
      //   //turns the robot left if tx is negative
      //     else if(m_limeLightBase.getTx()<-3){
      //   m_DriveBase.m_drive.arcadeDrive(0,-0.3);
      //   SmartDashboard.putString("Direction", "left");
      //   } 
  
      //   //does not turn the robot if tx is negligable :)
      //     else {
      //   m_DriveBase.m_drive.arcadeDrive(0,0);
      // }
  
      // }

      
  
    }

    public Command launchCommand(){
      return new Launch1to2Ball()
      .alongWith(new WaitCommand(0.1).andThen(new TopFeederActivate())
      .alongWith(new WaitCommand(0.3).andThen(new BottomFeederActivate())));
    }
   
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
