package org.usfirst.frc.team4913.robot;

import edu.wpi.first.wpilibj.Spark;

public class ShootingSystem {
	Spark shootingWheel;
	Spark spinningWheel;
	public ShootingSystem(int shootingWheelChannel,int spinningWheelChannel){
		shootingWheel = new Spark(shootingWheelChannel);
		spinningWheel = new Spark (spinningWheelChannel);
	}
	public void start(){
		shootingWheel.set(-1);
		spinningWheel.set(-0.4);
	}
	public void stop(){
		shootingWheel.set(0);
		spinningWheel.set(0);
	}
}
