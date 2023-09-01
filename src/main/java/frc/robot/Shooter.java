// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

/** Add your docs here. */
public class Shooter {
    private static Shooter instance_;
    
    public static Shooter getInstance() {
        if(instance_ == null) {
            instance_ = new Shooter();
        }
        return instance_;
    }


    
    TalonFX ShooterANG;
    TalonFX flyWheel;
    TalonFX shooter2;

    public Shooter() {
        ShooterANG = new TalonFX(6);
        flyWheel = new TalonFX(8); // guess port
        shooter2 = new TalonFX(13);
        ShooterANG.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);

        flyWheel.configVoltageCompSaturation(12);
        flyWheel.enableVoltageCompensation(true);

        shooter2.configVoltageCompSaturation(12);
        shooter2.enableVoltageCompensation(true);
        
    }

public void Shooter_angle(double input){
    ShooterANG.set(ControlMode.PercentOutput, input);
}
public void zeroSensor(){
    ShooterANG.setSelectedSensorPosition(0);
}
public double angle(){
    return ShooterANG.getSelectedSensorPosition(0);
}

//powers the flywheel that shoots out BIG balls
public void ShootMotor(double input) {
    flyWheel.set(ControlMode.PercentOutput, input);
   // flyWheel.set(TalonFXControlMode.Velocity, input);
}
public void ShootMotor2(double input) {
    shooter2.set(ControlMode.PercentOutput, -input);
    //shooter2.set(TalonFXControlMode.Velocity, -input);
}



//powers feeder wheel tha grabs the BIG balls into the BIG ball shooter


}
