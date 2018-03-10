/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* FRC Team 97: Bionic Beef                                                   */
/* Cambridge Rindge and Latin School                                          */
/*----------------------------------------------------------------------------*/

/*
 * Button Bindings:
 * - Right 6: Drive on/off
 * - Right X Axis: Not set
 * - Right Y Axis: Right Drive
 * - Left X Axis: Not set
 * - Left Y Axis: Left Drive
 * - Right 2: Right Draw Out
 * - Right 3: Right Draw In
 * - Left 2: Left Draw Out
 * - Left 3: Left Draw In
 * - Right 4: Right Drive Trim Down
 * - Right 5: Right Drive Trim Up
 * - Right 4: Left Drive Trim Down
 * - Right 5: Left Drive Trim Up
 */
// TODO: I still need to get the gyro working...

package org.usfirst.frc.team97.robot;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.wpilibj.ADXL362;

/**
 * This is the robot
 */
public class Robot extends IterativeRobot {

	// Differential drive = tank drive, used for both drive and shoot
	DifferentialDrive drive;
	DifferentialDrive shoot;
	DifferentialDrive shootX;

	// Outputs (motors)
	WPI_TalonSRX[] drive_motors;
	Spark[] shoot_motors;
	Spark[] shootX_motors;
	VictorSP[] draw_motors;

	// Inputs
	Joystick r_stick;
	Joystick l_stick;
	
	// delay used for drive and shoot on/off
	int bdelay_shoot;
	int bdelay_drive;

	double drive_spd;
	double draw_spd;

//	double drive_r_trim = 1;
//	double drive_l_trim = 1;
//	double shoot_r_trim = 1;
//	double shoot_l_trim = 1;
//	double shootX_r_trim = 1;
//	double shootX_l_trim = 1;
	
	double shoot_trim = 1;
	double shootX_trim = 1;

	double thresh = .2;

	// Acc
//	ADXL362 accmt;
	
	// Gyro
	ADXRS450_Gyro gyro;
	
	// Camera
	UsbCamera cam_serv_front;
	UsbCamera cam_serv_back;
	
	// Encoder
	Encoder enc;
	
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		cam_serv_front = CameraServer.getInstance().startAutomaticCapture(0);
		cam_serv_front.setResolution(1280/4, 720/4);
		cam_serv_back = CameraServer.getInstance().startAutomaticCapture(1);
		cam_serv_back.setResolution(1280/8, 720/8);

		drive_motors = new WPI_TalonSRX[4];

		for (int i = 0; i < 4; i++)
			(drive_motors[i] = new WPI_TalonSRX(i)).setSafetyEnabled(false);

		shoot_motors = new Spark[4];

		for (int i = 0; i < 4; i++)
			(shoot_motors[i] = new Spark(i)).setSafetyEnabled(false);

		shootX_motors = new Spark[2];

		for (int i = 0; i < 2; i++)
			(shootX_motors[i] = new Spark(i+6)).setSafetyEnabled(false);

		draw_motors = new VictorSP[4];

		for (int i = 0; i < 2; i++)
			(draw_motors[i] = new VictorSP(i + 4)).setSafetyEnabled(false);

		drive = new DifferentialDrive(new SpeedControllerGroup(drive_motors[1], drive_motors[3]),
				new SpeedControllerGroup(drive_motors[0], drive_motors[2]));
		shoot = new DifferentialDrive(new SpeedControllerGroup(shoot_motors[2], shoot_motors[3]),
				new SpeedControllerGroup(shoot_motors[0], shoot_motors[1]));

		shootX = new DifferentialDrive(shootX_motors[1], shootX_motors[0]);

		r_stick = new Joystick(0);
		l_stick = new Joystick(1);

		// SmartDashboard init
		SmartDashboard.putBoolean("Drive", true);
		
		SmartDashboard.putNumber("thresh", thresh);
		
		SmartDashboard.putBoolean("Shoot(X)", true);

		SmartDashboard.putString("Draw", "none");

//		SmartDashboard.putNumber("Drive Right Trim", drive_r_trim);
//		SmartDashboard.putNumber("Drive Left Trim", drive_l_trim);
//		SmartDashboard.putNumber("Shoot Right Trim", shoot_r_trim);
//		SmartDashboard.putNumber("Shoot Left Trim", shoot_l_trim);
//		SmartDashboard.putNumber("ShootX Right Trim", shootX_r_trim);
//		SmartDashboard.putNumber("ShootX Left Trim", shootX_l_trim);
		
		SmartDashboard.putNumber("Shoot Trim", shoot_trim * 100);
		SmartDashboard.putNumber("ShootX Trim", shootX_trim * 100);
		
		SmartDashboard.putNumber("time", 0);
		
		bdelay_shoot = bdelay_drive = 0;

		// Acc
//		accmt = new ADXL362(Accelerometer.Range.k2G);
		
		// Gyro
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		checkSense();
		
		// Encoder
		enc = new Encoder(1, 0);
		
		// Auto input vals
		SmartDashboard.putNumber("auto delay", 0);
		SmartDashboard.putString("start pos", "R");
		SmartDashboard.putNumber("auto speed", .6);
		SmartDashboard.putNumber("center capable", 0);
		SmartDashboard.putString("center force", "C");

	}

	// TODO: SET AUTO DATA HERE
	Auto auto;
	// Auto input vals
	String game_data;
	// Left: 'L', Center: 'C', Right: 'R'
	// Force right: 'R', Force left: 'L', No force: 'C'
	char low, high, start_pos = 'R', center_force = 'C';
	long auto_delay = 0; // delay for left/right
	double auto_speed = 0.6; // 0-1
	long center_capable = -1; // < 0 if they can low, > 0 for center delay overide
		
//	double acc;
	long last;
	long current;
	long start_time;
	double dist;
	double vel;
	double tar;
//	double acc_trim_x;
//	double acc_trim_y;
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		game_data = DriverStation.getInstance().getGameSpecificMessage();
		low = game_data.charAt(0);
		high = game_data.charAt(1);
//		auto_delay = (long) SmartDashboard.getNumber("auto delay", 0);
//		start_pos = SmartDashboard.getString("start_pos", "R").charAt(0);
//		auto_speed = SmartDashboard.getNumber("auto speed", .5);
//		center_capable = (long) SmartDashboard.getNumber("center capable", 0);
//		center_force = SmartDashboard.getString("center force", "C").charAt(0);
//		auto = new Auto(low, high, start_pos, auto_delay, auto_speed,
//		center_capable, center_force, drive, shoot, shootX, gyro,
//		enc, System.currentTimeMillis());
		auto = new Auto(low, high, start_pos, auto_delay, auto_speed,
		center_capable, center_force, drive, shoot, shootX, gyro,
		enc, System.currentTimeMillis());
		
		last = System.currentTimeMillis();
		tar = gyro.getAngle();
		dist = 0;
		vel = 0;
//		acc = getAcc();
//		calAcc(10);
		
		start_time = System.currentTimeMillis();
	}
	
//	private double getAcc() {
//		double mag = Math.pow(Math.pow(accmt.getX() - acc_trim_x, 2) + Math.pow(accmt.getY() - acc_trim_y, 2), 0.5);
//		return (accmt.getY() > 0) ? mag : -mag;
//		return accmt.getY() - acc_trim_y;
//	}
	
//	/**
//	 * Calibrate accelerometer
//	 * @return
//	 */
//	private void calAcc(int len) {
//		double sum_x = 0;
//		double sum_y = 0;
//		for(int i = 0; i < len; i++) {
//			sum_x += accmt.getX();
//			sum_y += accmt.getY();
//		}
//		acc_trim_x = sum_x/len;
//		acc_trim_y = sum_y/len;
//	}
	
	/*
	 * Auto Data:
	 * spd - time - dist
	 * .4  - 5000 - 46
	 * .5  - 5000 - 112.5 + ish
	 * .6  - 5000 - 16'2.5" + a little
	 * .7  - 3000 - 172"
	 * .8  - 2000 - 149"
	 * .9  - 1500 - 
	 * .7  - 5000 - 
	 * 
	 * Auto Data New and Improved:
	 * enc count / dist (in)
	 * 228/6m+5"
	 * 244/6m+4.25"
	 * 232/6m+6"
	 * 204/6m-1"
	 * 200/6m-1.5"
	 * 248/6m+7"
	 * Average = .9428 counts/in
	 */

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("auto path remaining", auto.run(System.currentTimeMillis()));
		
//		checkSense();
//		
//		acc = getAcc();
//		current = System.currentTimeMillis();
//		double time = current - last;
//		last = System.currentTimeMillis();
//		// Calcs
//		vel += time * acc;
//		dist += vel * time + .5 * acc * Math.pow(time, 2);
//		SmartDashboard.putNumber("dist", dist);
//		SmartDashboard.putNumber("vel", vel);
//		SmartDashboard.putNumber("acc", acc);
//		if(elapsed < 2000)
//			drive(SmartDashboard.getNumber("autoSpd", 0),SmartDashboard.getNumber("autoSpd", 0));
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		checkDrive();
		checkDraw();
		checkShoot();
		checkSense();
	}
	
	private void checkSense() {
		SmartDashboard.putNumber("Angle", gyro.getAngle());
//		SmartDashboard.putNumber("accX", accmt.getX());
//		SmartDashboard.putNumber("accY", accmt.getY());
//		SmartDashboard.putNumber("accZ", accmt.getZ());
	}

	private void checkShoot() {
		// Right side will trim up (5) and down (4) within range 0 to 1
//		if (r_stick.getRawButton(11) && shoot_r_trim < 1)
//			shoot_r_trim += .01;
//		if (r_stick.getRawButton(10) && shoot_r_trim > 0)
//			shoot_r_trim -= .01;
//		SmartDashboard.putNumber("Shoot Right Trim", shoot_r_trim);
//
//		// Left side will trim up (6) and down (7) within range 0 to 1
//		if (l_stick.getRawButton(6) && shoot_l_trim < 1)
//			shoot_l_trim += .01;
//		if (l_stick.getRawButton(7) && shoot_l_trim > 0)
//			shoot_l_trim -= .01;
//		SmartDashboard.putNumber("Shoot Left Trim", shoot_l_trim);
//
//		// Right side will trim up (9) and down (8) within range 0 to 1
//		if (r_stick.getRawButton(9) && shootX_r_trim < 1)
//			shootX_r_trim += .01;
//		if (r_stick.getRawButton(8) && shootX_r_trim > 0)
//			shootX_r_trim -= .01;
//		SmartDashboard.putNumber("ShootX Right Trim", shootX_r_trim);
//
//		// Left side will trim up (9) and down (8) within range 0 to 1
//		if (l_stick.getRawButton(9) && shootX_l_trim < 1)
//			shootX_l_trim += .01;
//		if (l_stick.getRawButton(8) && shootX_l_trim > 0)
//			shootX_l_trim -= .01;
//		SmartDashboard.putNumber("ShootX Left Trim", shootX_l_trim);

		
		// Shoot rim up (6) and down (7) within range 0 to 1
		if (l_stick.getRawButton(6) && shoot_trim < 1)
			shoot_trim += .01;
		if (l_stick.getRawButton(7) && shoot_trim > 0)
			shoot_trim -= .01;
		SmartDashboard.putNumber("Shoot Trim", shoot_trim * 100);

		// ShootX will trim up (9) and down (8) within range 0 to 1
		if (l_stick.getRawButton(9) && shootX_trim < 1)
			shootX_trim += .01;
		if (l_stick.getRawButton(8) && shootX_trim > 0)
			shootX_trim -= .01;
		SmartDashboard.putNumber("ShootX Trim", shootX_trim * 100);
		
		// Reverse
		if (SmartDashboard.getBoolean("Shoot(X)", false /*default*/) && l_stick.getRawButton(4)) {
			shoot.tankDrive(.7*shoot_trim, .7*shoot_trim);
			shootX.tankDrive(.7*shootX_trim, .7*shootX_trim);
		}

		// Shoot off/on
		if (r_stick.getRawButton(7) && bdelay_shoot++ > 8) {
			SmartDashboard.putBoolean("Shoot(X)", !SmartDashboard.getBoolean("Shoot(X)", false));
			bdelay_shoot = 0;
		}

		if (SmartDashboard.getBoolean("Shoot(X)", false /*default*/) && r_stick.getTrigger()) { // Shoot High
			shoot.tankDrive(-shoot_trim, -shoot_trim);
			shootX.tankDrive(-shootX_trim, -shootX_trim);
		}
		
		if (SmartDashboard.getBoolean("Shoot(X)", false /*default*/) && l_stick.getTrigger()) { // Shooter Low
			shoot.tankDrive(-.85 * shoot_trim, -.85 * shoot_trim);
			shootX.tankDrive(-.85 * shootX_trim, -.85 * shootX_trim);
		}
	}

	private void checkDrive() {
//		// Right side will trim up (5) and down (4) within range 0 to 1
//		if (r_stick.getRawButton(5) && drive_r_trim < 1)
//			drive_r_trim += .01;
//		if (r_stick.getRawButton(4) && drive_r_trim > 0)
//			drive_r_trim -= .01;
//		SmartDashboard.putNumber("Drive Right Trim", drive_r_trim);
//
//		// Light side will trim up (5) and down (4) within range 0 to 1
//		if (l_stick.getRawButton(5) && drive_l_trim < 1)
//			drive_l_trim += .01;
//		if (l_stick.getRawButton(4) && drive_l_trim > 0)
//			drive_l_trim -= .01;
//		SmartDashboard.putNumber("Drive Left Trim", drive_l_trim);

		
		if(r_stick.getRawButton(5))
			drive_spd = 1;
		else if(r_stick.getRawButton(4))
			drive_spd = .5;
		else
			drive_spd = (1 - r_stick.getRawAxis(2)) / 2;
		SmartDashboard.putNumber("Drive Speed", drive_spd * 100);

		// Drive off/on
		if (r_stick.getRawButton(6) && bdelay_drive++ > 8) {
			SmartDashboard.putBoolean("Drive", !SmartDashboard.getBoolean("Drive", false));
			bdelay_drive = 0;
		}
		
		thresh = SmartDashboard.getNumber("thresh", 0);
		if (SmartDashboard.getBoolean("Drive", false))
			drive(- (l_stick.getY() > 0 ? (thresh+l_stick.getY()/(1-thresh)) : (l_stick.getY()/(1-thresh)-thresh))   * drive_spd,
					-  (r_stick.getY() > 0 ? (thresh+r_stick.getY()/(1-thresh)) : (r_stick.getY()/(1-thresh)-thresh))   * drive_spd);
	}
	
	/**
	 * Drive based on left and right speeds
	 * Speed is squared
	 * 
	 * @param left - left speed (-1,1)
	 * @param right - right speed (-1,1)
	 */
	private void drive(double left, double right) {
		drive.tankDrive(left, right, true);
	}

	/**
	 * Drive at an angle to the gyro
	 * 
	 * @param spd - speed to drive
	 * @param gyro - AnalogGyro to base target
	 * @param angle - target angle (-1,1)
	 */
	protected void driveAngle(double spd, AnalogGyro gyro, double angle) {
		double adj = (gyro.getAngle() - angle)/180;
		drive.tankDrive(
				constrain(spd - adj),
				constrain(spd + adj));
	}
	
	protected static double constrain(double num, double max, double min) {
		return (num > max) ? 1 : (num < min) ? 0 : num;
	}
	
	static double constrain(double num) {
		return constrain(num, 1, 0);
	}
	
	private void checkDraw() {
		SmartDashboard.putNumber("Draw Speed Master", 100 * (draw_spd = (1 - l_stick.getRawAxis(2)) / 2));
		if (l_stick.getRawButton(2)) {
			draw_motors[1].set(1 * draw_spd);
			SmartDashboard.putString("Draw", "left in");
		} else if (l_stick.getRawButton(3)) {
			draw_motors[1].set(-1 * draw_spd);
			SmartDashboard.putString("Draw", "left out");
		} else {
			draw_motors[1].set(0);
			SmartDashboard.putString("Draw", "none");
		}
		if (r_stick.getRawButton(2)) {
			draw_motors[0].set(1 * draw_spd);
			SmartDashboard.putString("Draw", "right in");
		} else if (r_stick.getRawButton(3)) {
			draw_motors[0].set(-1 * draw_spd);
			SmartDashboard.putString("Draw", "right out");
		} else {
			draw_motors[0].set(0);
			SmartDashboard.putString("Draw", "none");
		}
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
