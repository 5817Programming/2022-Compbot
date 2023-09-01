/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Gyro {
    private static Gyro instance_;
	
	public static Gyro getInstance() {
		if(instance_ == null) {
			instance_ = new Gyro();
		}
		return instance_;
	}

	PigeonIMU _pidgey;
    
    TalonSRX pigeonTalon = new TalonSRX(1);

	public Gyro(){
		_pidgey = new PigeonIMU(pigeonTalon);
	}

    public double yaw(){
        return _pidgey.getYaw();
    }



}
