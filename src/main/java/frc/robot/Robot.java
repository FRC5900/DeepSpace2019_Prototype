/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* 1-29-19 Waiting for chassis design, adding compressor, solenoids           */  
/* 2-12-19 Testing Github                                                     */                                   
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/*OTHER CAMERA IMPORTS...
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
*/
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot 
{

  private SpeedController WheelIntakeMotor;
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private Joystick gamecontroller;


  Compressor c = new Compressor(0);
  boolean enabled = c.enabled();
  boolean pressureSwitch = c.getPressureSwitchValue();
  double current = c.getCompressorCurrent();

  //not too sure as to what we're going to use these for - Connor//
  Solenoid Valve1 = new Solenoid(0);
  Solenoid Valve2 = new Solenoid(1);
  Solenoid Valve3 = new Solenoid(2);
  Solenoid Valve4 = new Solenoid(3);
  Solenoid Valve5 = new Solenoid(4);
  Solenoid Valve6 = new Solenoid(5);
  Solenoid Valve7 = new Solenoid(6);
  Solenoid Valve8 = new Solenoid(7);

  double TurnSpeed;
  double DriveSpeed;
  double WheelIntakeSpeed;

  /* For Moving forward or reverse for number of seconds */
  int MoveRobot_State;
  int MoveRobot_Counter;
  int MoveRobot_TimeToMove;
  double MoveRobot_Speed;
  Boolean Forward;
  
  /* For Turning CW or CCW for number of seconds */
  int TurnRobot_State;
  int TurnRobot_Counter;
  int TurnRobot_TimeToMove;
  double TurnRobot_Speed;
  Boolean CW;
  
  /* For limiting speed of robot and removing voltage drift from joy stick */
  double RobotMaxSpeed = 0.75;
  double WheelIntakeMaxSpeed = 0.75;
  double DeadBand = 0.15;

  SpeedControllerGroup m_Left;
  SpeedControllerGroup m_Right;
      
  
  //original drive that worked was spark 1 was on 2 and 2 was on 1
  @Override
  public void robotInit() 
  {
    /*Spark is the type of motor driver, so it is what we call it - Connor*/
    /* Left drive requires two motors at channel 0, channel 1             */
    /* Right drive requires two motors at channel 2, channel 3            */
    m_Left = new SpeedControllerGroup( new Spark(0), new Spark(1));
    m_Right = new SpeedControllerGroup( new Spark(2), new Spark(3));
    m_myRobot = new DifferentialDrive(m_Left, m_Right);
    
    c.setClosedLoopControl(false);
    //set to true when ready ^
    //m_visionThread = new Thread(() -> {
    //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    //camera.setResolution(640, 480);
    
    WheelIntakeMotor = new Spark(4);
    /* Good job Connor */
    /*The order in which you plug in the joysticks, determines the port (0 = right, 1 = left) - Connor*/
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    MoveRobot_State = 0;
    MoveRobot_Speed = 0.0;
    TurnRobot_State = 0;
    SmartDashboard.putString( "what" , "RobotInit");     

    SmartDashboard.putNumber("MoveRobotState", MoveRobot_State);
    SmartDashboard.putNumber("TurnRobotState", TurnRobot_State);
  }

  @Override
  public void teleopInit() {
    super.teleopInit();

    SmartDashboard.putString( "what" , "teleopInit"); 
  }

  @Override
  public void teleopPeriodic() 
  {
    TurnSpeed = m_rightStick.getX();
    if (TurnSpeed > RobotMaxSpeed)
    TurnSpeed = RobotMaxSpeed;
    if (TurnSpeed < -RobotMaxSpeed)
      TurnSpeed = -RobotMaxSpeed;
    if ((TurnSpeed < DeadBand) && (TurnSpeed > -DeadBand))
      TurnSpeed = 0.0;

    DriveSpeed = m_rightStick.getY();
    if (DriveSpeed > RobotMaxSpeed)
      DriveSpeed = RobotMaxSpeed;
    if (DriveSpeed < -RobotMaxSpeed)
      DriveSpeed = -RobotMaxSpeed;
    if ((DriveSpeed < DeadBand) &&  (DriveSpeed > -DeadBand)) 
      DriveSpeed = 0.0;

    WheelIntakeSpeed = m_leftStick.getY();
    if (WheelIntakeSpeed > WheelIntakeMaxSpeed)
      WheelIntakeSpeed = WheelIntakeMaxSpeed;
    if (WheelIntakeSpeed < -WheelIntakeMaxSpeed)
      WheelIntakeSpeed = -WheelIntakeMaxSpeed;
    if ((WheelIntakeSpeed < DeadBand) &&  (WheelIntakeSpeed > -DeadBand)) 
      WheelIntakeSpeed = 0.0;

    if (( MoveRobot_State == 0 ) && ( TurnRobot_State == 0))
      m_myRobot.arcadeDrive(-DriveSpeed, TurnSpeed);
      
    SmartDashboard.putNumber("Left Speed", TurnSpeed);
    SmartDashboard.putNumber("Right Speed", DriveSpeed);
    SmartDashboard.putNumber("MoveRobotState", MoveRobot_State);
    SmartDashboard.putNumber("TurnRobotState", TurnRobot_State);

    /* Button 3 has it move forward - Connor */
    if ( m_rightStick.getRawButton(3) == true ) 
    {
      if ( MoveRobot_State == 0 ) 
      {
        MoveRobot_State = 1;
        Forward = true;
        MoveRobot_TimeToMove = 50;
        MoveRobot_Counter = 0;
        MoveRobot_Speed = 0.5;
        SmartDashboard.putString( "what" , "MoveRobot FWD");
      }
    }

    /* Button 2 has it move backwards - Connor */
    if ( m_rightStick.getRawButton(2) == true ) 
    {
      if ( MoveRobot_State == 0 ) 
      {
        MoveRobot_State = 1;
        Forward = false;
        MoveRobot_TimeToMove = 50;
        MoveRobot_Counter = 0;
        MoveRobot_Speed = 0.5;
        SmartDashboard.putString( "what" , "MoveRobot REV");
      }
    }

    WheelIntakeMotor.set(WheelIntakeSpeed);
    SmartDashboard.putNumber("WheelIntake", WheelIntakeSpeed);

    Move_Robot();

    /*Rotates Robot counter clockwise - Connor*/
    if ( m_rightStick.getRawButton(4) == true ) 
    {
      if ( TurnRobot_State == 0 ) 
      {
        TurnRobot_State = 1;
        CW = false;
        TurnRobot_TimeToMove = 100;
        TurnRobot_Counter = 0;
        TurnRobot_Speed = 0.5;
        SmartDashboard.putString( "what" , "TurnRobot CW");
      }
    }

    /*Rotates Robot clockwise - Connor*/
    if ( m_rightStick.getRawButton(5) == true ) 
    {
      if ( TurnRobot_State == 0 ) 
      {
        TurnRobot_State = 1;
        CW = true;
        TurnRobot_TimeToMove = 100;
        TurnRobot_Counter = 0;
        TurnRobot_Speed = 0.5;
        SmartDashboard.putString( "what" , "TurnRobot CCW");
      }
    }
    Turn_Robot();    
  }
      
  public void Move_Robot() 
  {
    switch (MoveRobot_State) 
    {
      case 1:
        if (MoveRobot_Counter > MoveRobot_TimeToMove) 
        {
          m_myRobot.tankDrive(0.0, 0.0);
          MoveRobot_Speed = 0.0;
          MoveRobot_State = 0;
          SmartDashboard.putString( "what" , "Turn Complete");
        }
        else 
        {
          if(Forward == true)
            m_myRobot.tankDrive(MoveRobot_Speed, MoveRobot_Speed);
          else
            m_myRobot.tankDrive(-MoveRobot_Speed, -MoveRobot_Speed);
          MoveRobot_Counter++;
        }
        break;
    }
  } 
 
  public void Turn_Robot() 
  {
    switch (TurnRobot_State) 
    {
      case 1:
        if (TurnRobot_Counter > TurnRobot_TimeToMove) 
        {
          m_myRobot.tankDrive(0.0, 0.0);
          TurnRobot_Speed = 0.0;
          TurnRobot_State = 0;
          SmartDashboard.putString( "what" , "Turn Complete");
        }
        else 
        {
          if ( CW == true)
            m_myRobot.tankDrive(TurnRobot_Speed, -TurnRobot_Speed);
          else
            m_myRobot.tankDrive(-TurnRobot_Speed, TurnRobot_Speed);
          TurnRobot_Counter++;
        }
    }
  }
 
  @Override
  public void testInit() {
    super.testInit();
    SmartDashboard.putString( "what" , "testInit"); 
  }
  @Override
  public void testPeriodic() 
  {
    m_Left.set(m_leftStick.getX());
    m_Right.set(m_rightStick.getX());
  }

}
