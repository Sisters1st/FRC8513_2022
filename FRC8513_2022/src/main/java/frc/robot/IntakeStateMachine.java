package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class IntakeStateMachine {

  private Rev2mDistanceSensor upperIntakeSensor;
  private Rev2mDistanceSensor lowerIntakeSensor;
  int intakeState;
  double upperSensorDistance;
  double lowerSensorDistance;
  int upperIntakeSensorTHold = 125;
  int lowerIntakeSensorTHold = 125;
  Robot thisRobot;

  public IntakeStateMachine(Robot thisRobotParameter) {
    thisRobot = thisRobotParameter;
  }

  public void intakeInit() {
    upperIntakeSensor = new Rev2mDistanceSensor(Port.kOnboard);
    lowerIntakeSensor = new Rev2mDistanceSensor(Port.kMXP);
  }

  public void setState(int wantedState) {
    intakeState = wantedState;
  }

  public void updateState()
  {
    try {
      upperSensorDistance=upperIntakeSensor.GetRange();
      lowerSensorDistance=lowerIntakeSensor.GetRange();
    }
    catch(Exception e) {
      upperSensorDistance=-1;
      lowerSensorDistance=-1;
    }
    switch(intakeState){
    case 0:
    break;
    case 1: //do nothing
      thisRobot.lowerMotorPower=0;
      thisRobot.upperMotorPower=0;
      thisRobot.flywheelMotor=0;
      if(thisRobot.joystick.getRawButtonPressed(1))
      {
        intakeState=3;
      }
      if(thisRobot.joystick.getRawButtonPressed(5))
      {
        intakeState=5;
      }
      if(thisRobot.joystick.getRawButtonPressed(5))
      {
        intakeState=2;
      }
    break;
    case 2: //both elevators on (dump)
      thisRobot.lowerMotorPower=1;
      thisRobot.upperMotorPower=1;
      thisRobot.flywheelMotor=1;
      if(thisRobot.joystick.getRawButtonReleased(5))
      {
        intakeState=1;
      }
    break;
    case 3: //both elevators on
      thisRobot.lowerMotorPower=1;
     thisRobot.upperMotorPower=1;
      thisRobot.flywheelMotor=0;
      if(thisRobot.joystick.getRawButtonPressed(4))
      {
        intakeState=1;
      }
      if(upperSensorDistance<upperIntakeSensorTHold)
      {
        intakeState=4;
      }
    break;
    case 4: //lower elevators on
     thisRobot.lowerMotorPower=1;
     thisRobot.upperMotorPower=0;
     thisRobot.flywheelMotor=0;
     if(thisRobot.joystick.getRawButtonPressed(4))
      {
        intakeState=1;
      }
      if(thisRobot.joystick.getRawButtonPressed(5))
      {
        intakeState=2;
      }
      if(lowerSensorDistance<lowerIntakeSensorTHold)
      {
        intakeState=1;
      }
    break;
    case 5: //forced intake state
      thisRobot.lowerMotorPower=1;
      thisRobot.upperMotorPower=1;
      thisRobot.flywheelMotor=0;
      if(thisRobot.joystick.getRawButtonReleased(5))
      {
        intakeState=1;
      }
    break;
    default:
    {   
      intakeState=1;
    }
    break;
  }
  SmartDashboard.putNumber("intake state", intakeState);
}

public void updateMotorPower()
{
  thisRobot.m_lowerIntakeMotor.set(thisRobot.lowerMotorPower);
  thisRobot.m_upperIntakeMotor.set(thisRobot.upperMotorPower);
  // set this later flywheelMotor if needed
  SmartDashboard.putNumber("lowerMotorPower", thisRobot.lowerMotorPower); 
  SmartDashboard.putNumber("upperMotorPower", thisRobot.upperMotorPower); 
}
}