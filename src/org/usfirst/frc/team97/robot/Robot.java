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
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXL362;

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

	double drive_r_trim = 1;
	double drive_l_trim = 1;
	double shoot_r_trim = 1;
	double shoot_l_trim = 1;
	double shootX_r_trim = 1;
	double shootX_l_trim = 1;
	
	double thresh = .2;

	// Acc
	ADXL362 acc;
	
	// Gyro
	ADXRS450_Gyro gyro;
	
	// Camera
	UsbCamera cam_serv;
	
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		cam_serv = CameraServer.getInstance().startAutomaticCapture();
		cam_serv.setResolution(1280/2, 720/2);

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
		SmartDashboard.putBoolean("Drive", false);
		
		SmartDashboard.putNumber("thresh", thresh);
		
		SmartDashboard.putBoolean("Shoot(X)", false);

		SmartDashboard.putString("Draw", "none");

		SmartDashboard.putNumber("Drive Right Trim", drive_r_trim);
		SmartDashboard.putNumber("Drive Left Trim", drive_l_trim);
		SmartDashboard.putNumber("Shoot Right Trim", shoot_r_trim);
		SmartDashboard.putNumber("Shoot Left Trim", shoot_l_trim);
		SmartDashboard.putNumber("ShootX Right Trim", shootX_r_trim);
		SmartDashboard.putNumber("ShootX Left Trim", shootX_l_trim);

		bdelay_shoot = bdelay_drive = 0;

		// Acc
		acc = new ADXL362(Accelerometer.Range.k4G);
		
		// Gyro
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		checkSense();
	}

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
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
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
		SmartDashboard.putNumber("accX", acc.getX());
		SmartDashboard.putNumber("accY", acc.getY());
		SmartDashboard.putNumber("accZ", acc.getZ());
	}

	private void checkShoot() {
		// Right side will trim up (5) and down (4) within range 0 to 1
		if (r_stick.getRawButton(11) && shoot_r_trim < 1)
			shoot_r_trim += .01;
		if (r_stick.getRawButton(10) && shoot_r_trim > 0)
			shoot_r_trim -= .01;
		SmartDashboard.putNumber("Shoot Right Trim", shoot_r_trim);

		// Left side will trim up (6) and down (7) within range 0 to 1
		if (l_stick.getRawButton(6) && shoot_l_trim < 1)
			shoot_l_trim += .01;
		if (l_stick.getRawButton(7) && shoot_l_trim > 0)
			shoot_l_trim -= .01;
		SmartDashboard.putNumber("Shoot Left Trim", shoot_l_trim);

		// Right side will trim up (9) and down (8) within range 0 to 1
		if (r_stick.getRawButton(9) && shootX_r_trim < 1)
			shootX_r_trim += .01;
		if (r_stick.getRawButton(8) && shootX_r_trim > 0)
			shootX_r_trim -= .01;
		SmartDashboard.putNumber("ShootX Right Trim", shootX_r_trim);

		// Left side will trim up (9) and down (8) within range 0 to 1
		if (l_stick.getRawButton(9) && shootX_l_trim < 1)
			shootX_l_trim += .01;
		if (l_stick.getRawButton(8) && shootX_l_trim > 0)
			shootX_l_trim -= .01;
		SmartDashboard.putNumber("ShootX Left Trim", shootX_l_trim);

		// Shoot off/on
		if (r_stick.getRawButton(7) && bdelay_shoot++ > 8) {
			SmartDashboard.putBoolean("Shoot(X)", !SmartDashboard.getBoolean("Shoot(X)", false));
			bdelay_shoot = 0;
		}

		if (SmartDashboard.getBoolean("Shoot(X)", false /*default*/) && r_stick.getTrigger()) { // Shoot High
			shoot.tankDrive(-shoot_l_trim, -shoot_r_trim);
			shootX.tankDrive(-shootX_l_trim, -shootX_r_trim);
		}
		if (SmartDashboard.getBoolean("Shoot(X)", false /*default*/) && l_stick.getTrigger()) { // Shooter Low
			shoot.tankDrive(-.7 * shoot_l_trim, -.7 * shoot_r_trim);
			shootX.tankDrive(-.7 * shootX_l_trim, -.7 * shootX_r_trim);
		}
	}

	private void checkDrive() {
		// Right side will trim up (5) and down (4) within range 0 to 1
		if (r_stick.getRawButton(5) && drive_r_trim < 1)
			drive_r_trim += .01;
		if (r_stick.getRawButton(4) && drive_r_trim > 0)
			drive_r_trim -= .01;
		SmartDashboard.putNumber("Drive Right Trim", drive_r_trim);

		// Light side will trim up (5) and down (4) within range 0 to 1
		if (l_stick.getRawButton(5) && drive_l_trim < 1)
			drive_l_trim += .01;
		if (l_stick.getRawButton(4) && drive_l_trim > 0)
			drive_l_trim -= .01;
		SmartDashboard.putNumber("Drive Left Trim", drive_l_trim);

		SmartDashboard.putNumber("Drive Speed", (drive_spd = (1 - r_stick.getRawAxis(2)) / 2));

		// Drive off/on
		if (r_stick.getRawButton(6) && bdelay_drive++ > 8) {
			SmartDashboard.putBoolean("Drive", !SmartDashboard.getBoolean("Drive", false));
			bdelay_drive = 0;
		}
		
		thresh = SmartDashboard.getNumber("thresh", 0);
		if (SmartDashboard.getBoolean("Drive", false))
			drive(- (l_stick.getY() > 0 ? (thresh+l_stick.getY()/(1-thresh)) : (l_stick.getY()/(1-thresh)-thresh))   * drive_spd * drive_l_trim,
					-  (r_stick.getY() > 0 ? (thresh+r_stick.getY()/(1-thresh)) : (r_stick.getY()/(1-thresh)-thresh))   * drive_spd * drive_r_trim);
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
	@SuppressWarnings("unused")
	private void driveAngle(double spd, AnalogGyro gyro, double angle) {
		double adj = (gyro.getAngle() - angle);
		drive.tankDrive(
				spd - adj,
				spd + adj);
	}
	
	private void checkDraw() {
		SmartDashboard.putNumber("Draw Speed Master", (draw_spd = (1 - l_stick.getRawAxis(2)) / 2));
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
			draw_motors[0].set(-1 * draw_spd);
			SmartDashboard.putString("Draw", "right in");
		} else if (r_stick.getRawButton(3)) {
			draw_motors[0].set(1 * draw_spd);
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
