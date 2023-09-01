// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/** Add your docs here. */
public class Elevator {
    private static Elevator instance_;
    public static Elevator getInstance() {
        if(instance_ == null) {
            instance_ = new Elevator();
        }
        return instance_;
        
    }
 TalonFX rightSideClimb;
 TalonFX leftSideClimb;
 Boolean leftLeveled = false;
 Boolean rightLeveled = false;

 public Elevator() {
     rightSideClimb = new TalonFX(2);
     leftSideClimb = new TalonFX(11);
 }



 public void elevatorMove (double input){
    rightSideClimb.set(ControlMode.PercentOutput, -input);
    leftSideClimb.set(ControlMode.PercentOutput, input);
 }

 public void leftElevMove (double input){
    leftSideClimb.set(ControlMode.PercentOutput, input);
 }

 public void rightElevMove (double input){
    rightSideClimb.set(ControlMode.PercentOutput, -input);
 }


 public double RightEncoderElevator(){
    return rightSideClimb.getSelectedSensorPosition();
}

public double LeftEncoderElevator(){
    return leftSideClimb.getSelectedSensorPosition();
}
public void zeroSensor(){
    rightSideClimb.setSelectedSensorPosition(0);
    leftSideClimb.setSelectedSensorPosition(0);
}

public boolean isLeftLeveled(double amperage){
    if (leftSideClimb.getStatorCurrent() > amperage){
        leftLeveled = true;
    }
    return leftLeveled;
}
public boolean isRightLeveled(double amperage){
    if (rightSideClimb.getStatorCurrent() > amperage){
        rightLeveled = true;
    }
    return rightLeveled;
}









}



