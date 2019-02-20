/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/*                                                                            */
/* 1-29-19 Waiting for chassis design, adding compressor, solenoids           */  
/* 2-12-19 Testing Github                                                     */   
/* Left Joystick at USB Port 0                                                */  
/* Right Joystick at USB Port 1                                               */                              
/*----------------------------------------------------------------------------*/
package frc.robot;

//Code list
//02-16-19 scrimmage--Worked
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//Extra camera imports to test ---Gage
/*import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import edu.wpi.first.cameraserver.CameraServer;
*/

public class Robot extends TimedRobot 
{
  private SpeedController BallIntakeMotor;
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  Compressor c = new Compressor(0);
 // boolean enabled = c.enabled();
 // boolean pressureSwitch = c.getPressureSwitchValue();
 // double current = c.getCompressorCurrent();

  
  DoubleSolenoid FrontLift = new DoubleSolenoid( 1, 0 );
  DoubleSolenoid RearLift = new DoubleSolenoid( 3, 2 );
  DoubleSolenoid HatchEject = new DoubleSolenoid( 4, 5 );
  DoubleSolenoid TieFighter = new DoubleSolenoid( 6, 7);
    

  double TurnSpeed;
  double DriveSpeed;
  double BallIntakeSpeed;
  double WinchSpeed;

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

  /* This is going to be for the Lifts */
  int FrontLift_State = 0;
  int RearLift_State = 0;
  
  /* This is for Autonomous Lift sequence */
  int AutoLift_State;
  int AutoLift_Counter;
  int AutoLift_TargetCount;
  int AutoLift_SaveState;

  /* For limiting speed of robot and removing voltage drift from joy stick */
  double RobotMaxSpeed = 0.75;
  double BallIntakeMaxSpeed = 1.0;
  double WinchMaxSpeed = 0.75;
  double DeadBand = 0.15;
  double JogSpeed = 0.25;
  double NormalSpeed = 0.75;

  SpeedControllerGroup m_Left;
  SpeedControllerGroup m_Right;
  SpeedControllerGroup m_Winch;

  //original drive that worked was spark 1 was on 2 and 2 was on 1
  @Override
  public void robotInit() 
  {
    
    /* Spark is the type of motor driver, so it is what we call it - Connor */
    /* Left drive requires two motors at channel 0, channel 1               */
    /* Right drive requires two moto rs at channel 2, channel 3             */
    m_Left = new SpeedControllerGroup( new Spark(0), new Spark(1));
    m_Right = new SpeedControllerGroup( new Spark(2), new Spark(3));
    m_Winch = new SpeedControllerGroup( new Spark(4), new Spark(5));
    m_myRobot = new DifferentialDrive(m_Left, m_Right);
    CameraServer.getInstance().startAutomaticCapture();
    c.setClosedLoopControl(true);
    

    /*m_visionThread = new Thread(() -> {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(640, 480);
    */
    
    BallIntakeMotor = new Spark(6);   
    //m_visionThread = new Thread(() -> {
    //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    //camera.setResolution(640, 480);

    BallIntakeMotor = new Spark(6);   
    /* Good job Connor */
>>>>>>> 7fddf415f20f8ce41e72ac51966c5e632339101a
    /*The order in which you plug in the joysticks, determines the port (0 = right, 1 = left) - Connor*/
    
   
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    MoveRobot_State = 0;
    MoveRobot_Speed = 0.0;
    TurnRobot_State = 0;
    
    FrontLift.set( DoubleSolenoid.Value.kOff );
    RearLift.set( DoubleSolenoid.Value.kOff );
    HatchEject.set( DoubleSolenoid.Value.kOff );
    
    SmartDashboard.putString("what" , "RobotInit");     
    SmartDashboard.putNumber("MoveRobotState", MoveRobot_State);
    SmartDashboard.putNumber("TurnRobotState", TurnRobot_State);
    
  }

  @Override
  public void autonomousInit() 
  {
    super.autonomousInit();
    SmartDashboard.putString( "what" , "autonomousInit"); 
  }


  @Override
  public void autonomousPeriodic()
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

    BallIntakeSpeed = m_leftStick.getY();
    if (BallIntakeSpeed > BallIntakeMaxSpeed)
      BallIntakeSpeed = BallIntakeMaxSpeed;
    if (BallIntakeSpeed < -BallIntakeMaxSpeed)
      BallIntakeSpeed = -BallIntakeMaxSpeed;
    if ((BallIntakeSpeed < DeadBand) &&  (BallIntakeSpeed > -DeadBand)) 
      BallIntakeSpeed = 0.0;

    if (( MoveRobot_State == 0 ) && ( TurnRobot_State == 0))
      m_myRobot.arcadeDrive(-DriveSpeed, TurnSpeed);
      
    SmartDashboard.putNumber("Left Speed", TurnSpeed);
    SmartDashboard.putNumber("Right Speed", DriveSpeed);
    SmartDashboard.putNumber("MoveRobotState", MoveRobot_State);
    SmartDashboard.putNumber("TurnRobotState", TurnRobot_State);
  
    /* If RightStick's button 3 is pressed, move forward - Connor */
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
  
    /* If RightStick's button 2 is pressed, move backwards - Connor */
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

    BallIntakeMotor.set(BallIntakeSpeed);
    SmartDashboard.putNumber("BallIntake", BallIntakeSpeed);

    Move_Robot();   // Process Move Robot request 

    /*Rotates Robot counter clockwise if RightStick's button 4 is pressed - Connor*/
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

    /*Rotates Robot clockwise if RightStick's button 5 is pressed - Connor*/
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
    Turn_Robot();    // Process Turn Robot request 

    /* Control Winch                                               */
    /*                                                             */
    if ( m_leftStick.getRawButton(3) == true ) 
       m_Winch.set( WinchMaxSpeed );
    else
    {
      if ( m_leftStick.getRawButton(2) == true )
        m_Winch.set( -WinchMaxSpeed );
      else
        m_Winch.set( 0.0 );
    }

  }


  @Override
  public void teleopInit() 
  {
    super.teleopInit();
    FrontLift_State = 0;
    RearLift_State = 0;
    SmartDashboard.putString( "what" , "teleopInit"); 
  }


  @Override
  public void teleopPeriodic() 
  {
    if ( FrontLift_State == 2 || RearLift_State == 2 )
      RobotMaxSpeed = JogSpeed;
    else
      RobotMaxSpeed = NormalSpeed;
      
    SmartDashboard.putNumber( "MaxSpeed" ,RobotMaxSpeed);

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

    BallIntakeSpeed = m_leftStick.getY();
    if (BallIntakeSpeed > BallIntakeMaxSpeed)
      BallIntakeSpeed = BallIntakeMaxSpeed;
    if (BallIntakeSpeed < -BallIntakeMaxSpeed)
      BallIntakeSpeed = -BallIntakeMaxSpeed;
    if ((BallIntakeSpeed < DeadBand) &&  (BallIntakeSpeed > -DeadBand)) 
      BallIntakeSpeed = 0.0;

    if (( MoveRobot_State == 0 ) && ( TurnRobot_State == 0))
      m_myRobot.arcadeDrive(-DriveSpeed, TurnSpeed);
      
    SmartDashboard.putNumber("Left Speed", TurnSpeed);
    SmartDashboard.putNumber("Right Speed", DriveSpeed);
    SmartDashboard.putNumber("MoveRobotState", MoveRobot_State);
    SmartDashboard.putNumber("TurnRobotState", TurnRobot_State);

    /* If RightStick's button 3 is pressed, move forward - Connor */
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
    /* If RightStick's button 2 is pressed, move backwards - Connor */
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
<<<<<<< HEAD
   
=======

>>>>>>> 7fddf415f20f8ce41e72ac51966c5e632339101a
    BallIntakeMotor.set(BallIntakeSpeed);
    SmartDashboard.putNumber("BallIntake", BallIntakeSpeed);

    Move_Robot();   // Process Move Robot request 

    /*Rotates Robot counter clockwise if RightStick's button 4 is pressed - Connor*/
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

    /*Rotates Robot clockwise if RightStick's button 5 is pressed - Connor*/
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
    Turn_Robot();    // Process Turn Robot request 

    /* Control Winch                                               */
    /*                                                             */
    if ( m_leftStick.getRawButton(3) == true ) 
       m_Winch.set( -WinchMaxSpeed );
    else
    {
      if ( m_leftStick.getRawButton(2) == true )
        m_Winch.set( WinchMaxSpeed );
      else
        m_Winch.set( 0.0 );
    }

<<<<<<< HEAD
=======
    if( m_leftStick.getRawButton(4) )
      FrontLift.set( DoubleSolenoid.Value.kForward );
    else
      FrontLift.set( DoubleSolenoid.Value.kReverse );

    if( m_leftStick.getRawButton(5) )
      RearLift.set( DoubleSolenoid.Value.kForward );
    else
      RearLift.set( DoubleSolenoid.Value.kReverse );
>>>>>>> 7fddf415f20f8ce41e72ac51966c5e632339101a

    if( m_leftStick.getRawButton(1) )
      HatchEject.set( DoubleSolenoid.Value.kForward );
    else
      HatchEject.set( DoubleSolenoid.Value.kReverse );
<<<<<<< HEAD

    if( m_leftStick.getRawButton(8) )
      TieFighter.set( DoubleSolenoid.Value.kForward );
    else
      TieFighter.set( DoubleSolenoid.Value.kReverse );

    Lifts_Control();
=======
>>>>>>> 7fddf415f20f8ce41e72ac51966c5e632339101a
  }
 
  /* Move_Robot() - will move robot FWD or REV for given duration  */
  /*                                                               */
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
 

  /* Turn_Robot() - will turn robot CW or CCW for given duration  */
  /*                                                              */
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
  public void testInit() 
  {
    super.testInit();
    SmartDashboard.putString( "what" , "testInit"); 
  }


  @Override
  public void testPeriodic() 
  {
    m_Left.set(m_leftStick.getY());
    m_Right.set(m_rightStick.getY());



    if( m_rightStick.getRawButton(3) )
      FrontLift.set( DoubleSolenoid.Value.kForward );
    else
      FrontLift.set( DoubleSolenoid.Value.kReverse );

    if( m_rightStick.getRawButton(2) )
      RearLift.set( DoubleSolenoid.Value.kForward );
    else
      RearLift.set( DoubleSolenoid.Value.kReverse );

    if( m_rightStick.getRawButton(1) )
      HatchEject.set( DoubleSolenoid.Value.kForward );
    else
      HatchEject.set( DoubleSolenoid.Value.kReverse );
  

  }

  public void Lifts_Control()
  {
    switch(FrontLift_State)
    {
      case 0:
        if (m_leftStick.getRawButton(4) == true)
        {
          FrontLift.set( DoubleSolenoid.Value.kForward );
          SmartDashboard.putString( "lifts F" , "Extend FrontLift"); 
          FrontLift_State = 1;
        }
        break;

      case 1:
        if (m_leftStick.getRawButton(4) == false)
          FrontLift_State = 2;
        break;

      case 2:
        if (m_leftStick.getRawButton(4) == true)
        {
          FrontLift.set( DoubleSolenoid.Value.kReverse );
          SmartDashboard.putString( "lifts F" , "Retract FrontLift");
          FrontLift_State = 3;
        }
        break;

      case 3:
        if (m_leftStick.getRawButton(4) == false)
          FrontLift_State = 0;
        break;  

    }

    switch(RearLift_State)
    {
      case 0:
        if (m_leftStick.getRawButton(5) == true)
        {
          RearLift.set( DoubleSolenoid.Value.kForward );
          SmartDashboard.putString( "lifts R" , "Extend RearLift" );
          RearLift_State = 1;
        }
        break;

      case 1:
        if (m_leftStick.getRawButton(5) == false)
          RearLift_State = 2;
        break;

      case 2:
        if (m_leftStick.getRawButton(5) == true)
        {
          RearLift.set( DoubleSolenoid.Value.kReverse );
          SmartDashboard.putString( "lifts R" , "Retract RearLift");
          RearLift_State = 3;
        }
        break;

      case 3:
        if (m_leftStick.getRawButton(5) == false)
          RearLift_State = 0;
        break;  
    }
  }

  public void AutoLift_Controller()
  {
    if (m_leftStick.getRawButton(9) == true)   // Stop Sequence 
    {
      if( AutoLift_State < 100 )
      {
        m_myRobot.arcadeDrive(0.0, 0.0);
        AutoLift_SaveState = AutoLift_State;
        AutoLift_State = 100;
        SmartDashboard.putString( "AutoLift" , "Suspending Lift");
      }  
    }

    switch (AutoLift_State)
    {
      case 0: //Waiting for button 5 to be pressed
        if (m_leftStick.getRawButton(5) == true)
        {
          AutoLift_State = 1;
          AutoLift_TargetCount = 25;
          AutoLift_Counter = 0;
          FrontLift.set( DoubleSolenoid.Value.kForward );
          SmartDashboard.putString( "AutoLift" , "Extend FrontLift"); 
        }
       
        break;

      case 1: //Waiting for front lift to extend
        if (++AutoLift_Counter > AutoLift_TargetCount)
        {
          AutoLift_State = 2;
          AutoLift_TargetCount = 25;
          AutoLift_Counter = 0;
          m_myRobot.tankDrive(0.25, 0.25);
          SmartDashboard.putString( "AutoLift" , "Move Forward at 25% Speed"); 
        }
        break;

      case 2: //Wating for robot to move for (half a second)
        if (++AutoLift_Counter > AutoLift_TargetCount)
        {
          m_myRobot.tankDrive(0.0, 0.0);
          AutoLift_State = 3;
          AutoLift_TargetCount = 25;
          AutoLift_Counter = 0;
          FrontLift.set( DoubleSolenoid.Value.kReverse );
          SmartDashboard.putString( "AutoLift" , "Retract FrontLift"); 
        }
        break;

      case 3: //Wating for front lift to retract
        if (++AutoLift_Counter > AutoLift_TargetCount)
        {
          RearLift.set( DoubleSolenoid.Value.kForward );
          AutoLift_State = 4;
          AutoLift_TargetCount = 25;
          AutoLift_Counter = 0;
          SmartDashboard.putString( "AutoLift" , "Extend RearLift"); 
        }
        break;

      case 4: //Waiting for rear lift to activate
        if (++AutoLift_Counter > AutoLift_TargetCount)
        {
          AutoLift_State = 5;
          AutoLift_TargetCount = 25;
          AutoLift_Counter = 0;
          m_myRobot.tankDrive(0.25, 0.25);
          SmartDashboard.putString( "AutoLift" , "Move Forward at 25% Speed"); 
        }
        break;

      case 5: //Wating for robot to move for half a second
        if (++AutoLift_Counter > AutoLift_TargetCount)
        {
          AutoLift_State = 6;
          AutoLift_TargetCount = 25;
          AutoLift_Counter = 0;
          m_myRobot.tankDrive(0.0, 0.0);
          RearLift.set ( DoubleSolenoid.Value.kReverse );
          SmartDashboard.putString( "AutoLift" , "Retract RearLift");
        }
        break;

      case 6: //Waiting for the rear lift to retract
        if (++AutoLift_Counter > AutoLift_TargetCount)
        {
          AutoLift_State = 7;
          AutoLift_TargetCount = 0;
          AutoLift_Counter = 0;
          m_myRobot.tankDrive(0.0, 0.0);
          FrontLift.set ( DoubleSolenoid.Value.kReverse );
          RearLift.set ( DoubleSolenoid.Value.kReverse );
          SmartDashboard.putString( "AutoLift" , "AutoLift Complete");
        }
        break;
    
      case 7: // Auto Lift Complete, just idle here
        break;

      case 100: 
        if (m_leftStick.getRawButton(9) == false)   // Stop Sequence 
        {
           m_myRobot.arcadeDrive(0.0, 0.0);
           AutoLift_State = 101;
           SmartDashboard.putString( "AutoLift" , "Wait for Resume");
        }
        break;

      case 101:
        if (m_leftStick.getRawButton(9) == true)   // Resume Sequence 
        {
           m_myRobot.arcadeDrive(0.0, 0.0);
           AutoLift_State = AutoLift_SaveState;
           SmartDashboard.putString( "AutoLift" , "Resume AutoLift");
        }  
        break;

      default:
        AutoLift_State = 0;
        break;
    }
  }
}


