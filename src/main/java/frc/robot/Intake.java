// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/** Add your docs here. */
public class Intake {
    private static Intake instance_;
    public static Intake getInstance() {
        if(instance_ == null) {
            instance_ = new Intake();
        }
        return instance_;
    }

    TalonSRX BallsGrabber;
    TalonSRX Indexer;
    TalonFX feeder;

public Intake() {
    BallsGrabber = new TalonSRX(12); //change number later
    Indexer = new TalonSRX(1);
    feeder = new TalonFX(7);

    SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);
    Indexer.configSupplyCurrentLimit(supplyLimit);
  

    StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 60, 80, 0.05);
    //Indexer.configStatorCurrentLimit(statorLimit);

   

    Indexer.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
    }   


// powers the indexer and intake that pulls BIG balls into the BIG ball feeder
public void BallsGrabber(double input){ 
    BallsGrabber.set(ControlMode.PercentOutput, input);
      // power 0.85
}

public void towerIntake(double input){
    Indexer.set(ControlMode.PercentOutput, input);
}

public void feeder(double input){
    feeder.set(ControlMode.PercentOutput, input);
    
}

    
}
