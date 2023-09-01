/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
/**
 * Add your docs here.
 */
public class Drive {
    public static final int Ktick2feet = 0;
    private static Drive instance_;
    
    public static Drive getInstance() {
        if(instance_ == null) {
            instance_ = new Drive();
        }
        return instance_;
    }

    TalonFX leftDriveMaster;
    TalonFX rightDriveMaster;

    TalonFX leftDriveSlave;
    TalonFX rightDriveSlave;

    List<TalonFX> motors;

    PigeonIMU pidgey;
    TalonSRX pigeontalon; 




 public double kTick2feet = (((4 * Math.PI)/2048)/12)/  7; 
 //Wheel circumference / encoder tick per revolution, converted into feet, and divided by the gear ratio.
public int rightEncoder;

    

    public Drive() {
        leftDriveMaster = new TalonFX(9);
        rightDriveMaster = new TalonFX(3);

        leftDriveSlave = new TalonFX(10);
        rightDriveSlave = new TalonFX(4);

        leftDriveSlave.follow(leftDriveMaster);
        rightDriveSlave.follow(rightDriveMaster);

        SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);
        leftDriveMaster.configSupplyCurrentLimit(supplyLimit);
        leftDriveSlave.configSupplyCurrentLimit(supplyLimit);
        rightDriveMaster.configSupplyCurrentLimit(supplyLimit);
        rightDriveSlave.configSupplyCurrentLimit(supplyLimit);

        StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 60, 80, 0.05);
        leftDriveMaster.configStatorCurrentLimit(statorLimit);
        leftDriveSlave.configStatorCurrentLimit(statorLimit);
        rightDriveMaster.configStatorCurrentLimit(statorLimit);
        rightDriveSlave.configStatorCurrentLimit(statorLimit);




        rightDriveMaster.setInverted(false);
      //  leftDriveMaster.setInverted(true);

        
        leftDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
        rightDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);

        pigeontalon = new TalonSRX(5);
        pidgey = new PigeonIMU(pigeontalon);


    }


    public void RightDrive(double speed){
        rightDriveMaster.set(ControlMode.PercentOutput, speed);
    }
    
    public void LeftDrive (double speed) {
        leftDriveMaster.set (ControlMode.PercentOutput, -speed);
    }

    public double LeftEncoder(){
        return leftDriveMaster.getSelectedSensorPosition();
    }
    public double RightEncoder(){
        return rightDriveMaster.getSelectedSensorPosition(0);
    }
    public void zeroSensor(){
        rightDriveMaster.setSelectedSensorPosition(0);
        leftDriveMaster.setSelectedSensorPosition(0);
    }
    public double getYaw(){
        return pidgey.getYaw();
    }



}