// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.midi.SysexMessage;
import javax.swing.plaf.basic.BasicBorders.SplitPaneBorder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.SuperJoystick;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final int  FLangleID = 8;
  private static final int FLdriveID = 7;
  private static final int  FRangleID = 6;
  private static final int FRdriveID = 5;
  private static final int  BRangleID = 4;
  private static final int BRdriveID = 3;
  private static final int  BLangleID = 2;
  private static final int BLdriveID = 1;
  private CANSparkMax FLangleMotor;
  private CANSparkMax FLdriveMotor;
  private CANSparkMax FRdriveMotor;
  private CANSparkMax FRangleMotor;
  private CANSparkMax BLangleMotor;
  private CANSparkMax BLdriveMotor;
  private CANSparkMax BRdriveMotor;
  private CANSparkMax BRangleMotor;
  private SparkMaxPIDController m_PIDController1;
  private SparkMaxPIDController m_PIDController2;
  private SparkMaxPIDController m_PIDController3;
  private SparkMaxPIDController m_PIDController4;
  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;
  private RelativeEncoder m_encoder3;
  private RelativeEncoder m_encoder4;
  private XboxController drive_control;
  private Timer autoTimer;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  // MAX SPEED
  private int SpeedLimiter;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
        drive_control = new XboxController(0);
        // initialize motor
        FLangleMotor = new CANSparkMax(FLangleID, MotorType.kBrushless);
        FLdriveMotor = new CANSparkMax(FLdriveID, MotorType.kBrushless);
        FRangleMotor = new CANSparkMax(FRangleID, MotorType.kBrushless);
        FRdriveMotor = new CANSparkMax(FRdriveID, MotorType.kBrushless);
        BRangleMotor = new CANSparkMax(BRangleID, MotorType.kBrushless);
        BRdriveMotor = new CANSparkMax(BRdriveID, MotorType.kBrushless);
        BLdriveMotor = new CANSparkMax(BLdriveID, MotorType.kBrushless);
        BLangleMotor = new CANSparkMax(BLangleID, MotorType.kBrushless);
        /**
         * The restoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */


    
        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_PIDController1 = FLangleMotor.getPIDController();
        m_PIDController2 = FRangleMotor.getPIDController();
        m_PIDController3 = BRangleMotor.getPIDController();
        m_PIDController4 = BLangleMotor.getPIDController();


    
        // Encoder object created to display position values
        m_encoder1 = FLangleMotor.getEncoder();
        m_encoder2 = FRangleMotor.getEncoder();
        m_encoder3 = BRangleMotor.getEncoder();
        m_encoder4 = BLangleMotor.getEncoder();

        // Auto Timer
        autoTimer = new Timer();
    
        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
    
        // set PID coefficients
        m_PIDController1.setP(kP);
        m_PIDController2.setP(kP);
        m_PIDController3.setP(kP);
        m_PIDController4.setP(kP);

        m_PIDController1.setI(kI);
        m_PIDController2.setI(kI);
        m_PIDController3.setI(kI);
        m_PIDController4.setI(kI);

        m_PIDController1.setD(kD);
        m_PIDController2.setD(kD);
        m_PIDController3.setD(kD);
        m_PIDController4.setD(kD);

        m_PIDController1.setIZone(kIz);
        m_PIDController2.setIZone(kIz);
        m_PIDController3.setIZone(kIz);
        m_PIDController4.setIZone(kIz);

        m_PIDController1.setFF(kFF);
        m_PIDController2.setFF(kFF);
        m_PIDController3.setFF(kFF);
        m_PIDController4.setFF(kFF);


        m_PIDController1.setOutputRange(kMinOutput, kMaxOutput);
        m_PIDController2.setOutputRange(kMinOutput, kMaxOutput);
        m_PIDController3.setOutputRange(kMinOutput, kMaxOutput);
        m_PIDController4.setOutputRange(kMinOutput, kMaxOutput);
    
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
        SmartDashboard.putNumber("Run Speed Motor?", 0);
        SmartDashboard.putBoolean("Joystick Control", false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoTimer.reset();
    autoTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom 
        break;
      case kDefaultAuto:
      default:
        if(autoTimer.get() < 1.5) {
          FRdriveMotor.set(.15);
          FLdriveMotor.set(.15);
          BRdriveMotor.set(-.15);
          BLdriveMotor.set(-.15);
        }
        else if(autoTimer.get() < 3) {
          FRdriveMotor.set(-.15);
          FLdriveMotor.set(-.15);
          BRdriveMotor.set(.15);
          BLdriveMotor.set(.15);
        }
        else if(autoTimer.get() > 3) {
          FRdriveMotor.set(0);
          FLdriveMotor.set(0);
          BRdriveMotor.set(0);
          BLdriveMotor.set(0);  
        }
        else {
          FRdriveMotor.set(0);
          FLdriveMotor.set(0);
          BRdriveMotor.set(0);
          BLdriveMotor.set(0);
        }
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);
        double Speed_Motor = SmartDashboard.getNumber("Run Speed Motor?", 0);
        boolean drive_mode = SmartDashboard.getBoolean("Joystick Control", false);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { 
          m_PIDController1.setP(p); kP = p; 
          m_PIDController2.setP(p); kP = p; 
          m_PIDController3.setP(p); kP = p; 
          m_PIDController4.setP(p); kP = p; 
        }
        if((i != kI)) { 
          m_PIDController1.setI(i); kI = i; 
          m_PIDController2.setI(i); kI = i; 
          m_PIDController3.setI(i); kI = i; 
          m_PIDController4.setI(i); kI = i; 
        }
        if((d != kD)) { 
          m_PIDController1.setD(d); kD = d; 
          m_PIDController2.setD(d); kD = d; 
          m_PIDController3.setD(d); kD = d; 
          m_PIDController4.setD(d); kD = d; 
        }
        if((iz != kIz)) {
           m_PIDController1.setIZone(iz); kIz = iz; 
           m_PIDController2.setIZone(iz); kIz = iz; 
           m_PIDController3.setIZone(iz); kIz = iz; 
           m_PIDController4.setIZone(iz); kIz = iz;           
          }
        if((ff != kFF)) { 
          m_PIDController1.setFF(ff); kFF = ff; 
          m_PIDController2.setFF(ff); kFF = ff; 
          m_PIDController3.setFF(ff); kFF = ff; 
          m_PIDController4.setFF(ff); kFF = ff; 
        }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_PIDController1.setOutputRange(min, max); 
          m_PIDController2.setOutputRange(min, max); 
          m_PIDController3.setOutputRange(min, max); 
          m_PIDController4.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
       // if(drive_mode == false) {
         // FLdriveMotor.set(Math.abs(Speed_Motor));
         // FRdriveMotor.set(Math.abs(Speed_Motor));
         // BRdriveMotor.set(Speed_Motor);
         // BLdriveMotor.set(Speed_Motor);
       // }




       FLdriveMotor.set((drive_control.getLeftY() * -1) / 4);
       FRdriveMotor.set((drive_control.getLeftY() * -1) / 4);
       BRdriveMotor.set(drive_control.getLeftY() / 4);
       BLdriveMotor.set(drive_control.getLeftY() / 4);

        // if(drive_mode = true) {
       m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
       m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
       m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
       m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
       SmartDashboard.putNumber("SetPoint", rotations);
       SmartDashboard.putNumber("ProcessVariable", m_encoder1.getPosition());
       // }

      

       //BRdriveMotor.set(drive_control.getLeftY() / 4);
       //BRdriveMotor.set(drive_control.getLeftX() / 4);
       //BLdriveMotor.set(drive_control.getLeftY() / 4);
       //BLdriveMotor.set(drive_control.getLeftX() / 4);
      
       if(drive_control.getRightX() > .005) {
          m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set(.25);
          m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
          FRdriveMotor.set(-.25);
          m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
          BLdriveMotor.set(-.25);
          m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
          BRdriveMotor.set(.25);
       }
       // Rotation
       if(drive_control.getRightX() < -.005) {
        m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
        FLdriveMotor.set(-.25);
        m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
        FRdriveMotor.set(.25);
        m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
        BLdriveMotor.set(.25);
        m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
        BRdriveMotor.set(-.25);
      }
      if(drive_control.getRawAxis(0) == 1) {
        m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
        FLdriveMotor.set(- .25);
        m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
        FRdriveMotor.set(- .25);
        m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
        BLdriveMotor.set(.25);
        m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
        BRdriveMotor.set(.25);

      }
      if(drive_control.getRawAxis(0) == -1) {
        m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
        FLdriveMotor.set(.25);
        m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
        FRdriveMotor.set(.25);
        m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
        BLdriveMotor.set(- .25);
        m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
        BRdriveMotor.set(- .25);
      }
       while(drive_control.getRightTriggerAxis() > .05) {
        m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
        m_PIDController2.setReference(-2,  CANSparkMax.ControlType.kPosition);
        FLdriveMotor.set((drive_control.getLeftY() * -1));
        FRdriveMotor.set((drive_control.getLeftY() * -1));
        BRdriveMotor.set(drive_control.getLeftY());
        BLdriveMotor.set(drive_control.getLeftY());
        

      }
      while(drive_control.getLeftTriggerAxis() > .05) {
        m_PIDController1.setReference(2,  CANSparkMax.ControlType.kPosition);
        m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
        FLdriveMotor.set((drive_control.getLeftY() * -1));
        FRdriveMotor.set((drive_control.getLeftY() * -1));
        BRdriveMotor.set(drive_control.getLeftY());
        BLdriveMotor.set(drive_control.getLeftY());
      }
      
      while(drive_control.getLeftBumper() == true) {
        m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
        m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
        m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
        m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
        FLdriveMotor.set(drive_control.getRightX() / 15);
        FRdriveMotor.set((drive_control.getRightX() * -1) / 15);
        BRdriveMotor.set(drive_control.getRightX() / 15);
        BLdriveMotor.set((drive_control.getRightX() * -1) / 15);
      }
        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        // if(drive_mode == false) {
        //  m_PIDController1.setReference(rotations, CANSparkMax.ControlType.kPosition);
        //  m_PIDController2.setReference(rotations, CANSparkMax.ControlType.kPosition);
        //  m_PIDController3.setReference(rotations, CANSparkMax.ControlType.kPosition);
        //  m_PIDController4.setReference(rotations, CANSparkMax.ControlType.kPosition);
        //}
        //if(drive_mode == true) {

        //}
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
