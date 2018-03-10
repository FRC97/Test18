package org.usfirst.frc.team97.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {
	
	protected static final long
	delay = 0,
	move_enc = 1,
	turn = 2,
	shoot = 3,
	reverse = 4,
	move_enc_gyro = 5, // move straight with the gyro + enc
	move_time_gyro = 6, // move straight with the gyro + time
	move_time = 7; // move straight with time
	
	protected static final long cmp = 1, inc = 0; // complete, incomplete
	
	protected static final long shoot_low = 1, shoot_high = 2;
	
	// Auto distances -- inches
	protected static final long
	base_side_offset = 40,
	side_to_low = 110, // 109.82 at an angle
	side_to_high = 350, // 193.25 at an angle
	base_to_first_center = 40,
	base_to_second_mid = 275,
	first_center_to_first_sideR = 110,
	first_center_to_first_sideL = 120,
	first_side_to_low = 109, // 108.5
	low_to_fence = 35, // 17.75
	second_mid_to_center = 100,
	high_turn_ang = 5, // 5.07
	low_turn_ang = 9; // 9.05
	
	private static final long shoot_time = 1000;
	
	// [[completed, action type, action value][...]]
	List<long[]> path = new ArrayList<long[]>();
	private double spd;
	private long cmd_start_time;
	private double cmd_start_angle;
	private long cmd_start_enc;
	
	private static final double
	low_shoot_spd = .8,
	high_shoot_spd = 1,
	turn_spd = .55,
	conv_count_to_in = 1, //0.9428;
	in_per_millis = 25.47;
	
	DifferentialDrive drive, shootL, shootX;
	ADXRS450_Gyro gyro;
	Encoder enc;
	
	/**
	 * 
	 * @param low: low goal 'R' or 'L'
	 * @param high: high goal 'R' or 'L'
	 * @param start_pos: 'L', 'C', 'R'
	 * @param auto_delay: always first, in ms
	 * @param auto_speed: speed throughout
	 * @param center_capable: <0 means it can do low, >=0 ms extra delay if going low
	 * @param center_force: if we are center, always go to this side
	 */
	public Auto(char low, char high, char start_pos, long auto_delay, double auto_speed, long center_capable,
			char center_force, DifferentialDrive drive, DifferentialDrive shootL, DifferentialDrive shootX,
			ADXRS450_Gyro gyro, Encoder enc, long now) {
		spd = auto_speed;
		this.drive = drive;
		this.shootL = shootL;
		this.shootX = shootX;
		this.gyro = gyro;
		this.enc = enc;
		
		long move_mode = move_time_gyro;

		reset_count(now);
		
//		addToPath(turn, 90);
		
//		Test Auto Run
//		addToPath(move_mode, (long) SmartDashboard.getNumber("time", 0));
//		addToPath(delay, 10000);
//		addToPath(shoot, shoot_low);
		
		addToPath(delay, auto_delay);
				
		//Center
		if(start_pos == 'C') {
			addToPath(move_mode, base_to_first_center);
			char dir = center_force == 'C' ? low : center_force;
			turnOut(dir, 90);
			if(dir == 'R')
				addToPath(move_mode, first_center_to_first_sideR);
			else
				addToPath(move_mode, first_center_to_first_sideL);				
			turnIn(dir, 90);
			addToPath(move_mode, first_side_to_low);
			if(low == dir) { // We're going towards the low goal -> shoot
				turnOut(dir, 70);
				addToPath(reverse, low_to_fence);
				addToPath(delay, 300);
				addToPath(shoot, shoot_low);
			}
		}
		
		// Right/Left
		else {
			// If high is on our side and either there's no low or center can low -> High goal
			if(start_pos == high && (start_pos != low || center_capable < 0))
			{
				addToPath(move_mode, base_side_offset);
//				turnOut(start_pos, high_turn_ang);
				addToPath(move_mode, side_to_high);
				turnOut(start_pos, 90 - 0/*high_turn_ang*/);
//				addToPath(delay, 100);
				addToPath(delay, 300);
				addToPath(shoot, shoot_high);
			}
			
			// If low is open and center cannot low (and not going high) -> Low goal
			else if(start_pos == low && center_capable >= 0) {
				if(center_capable > 0) addToPath(delay, center_capable);
				addToPath(move_mode, base_side_offset);
				turnOut(start_pos, low_turn_ang);
				addToPath(move_mode, side_to_low);
				turnOut(start_pos, 90 - low_turn_ang);
				addToPath(reverse, low_to_fence);
				addToPath(delay, 300);
				addToPath(shoot, shoot_low);
			}
			
			// None or low only and center can low -> Center middle (get out of the way)
			else {
				addToPath(move_mode, base_to_second_mid);
				turnIn(start_pos, 90);
				addToPath(move_mode, second_mid_to_center);
			}
		}
	}
	
	private void addToPath(long action, long value) {
		path.add(new long[] {inc, action, value});
	}
	
	private void turnOut(char side) {
		addToPath(turn, side == 'R' ? 90: -90);
	}

	private void turnIn(char side) {
		addToPath(turn, side == 'R' ? -90: 90);
	}
	
	private void turnOut(char side, long ang) {
		addToPath(turn, side == 'R' ? ang: -ang);
	}

	private void turnIn(char side, long ang) {
		addToPath(turn, side == 'R' ? -ang: ang);
	}
	
	protected String run(long now) {		
		if(path.size() > 0) {
			long[] command = path.get(0);
			double adj = 0;
			switch((int) command[1]) {
				case (int) delay:
					if(now - cmd_start_time > command[2]) rmFromPath(now);
					break;
					
				case (int) turn:
					if(command[2] > 0) {
						drive.tankDrive(turn_spd, -turn_spd);
						if(gyro.getAngle() > cmd_start_angle + command[2]) rmFromPath(now);
					}
					else {
						drive.tankDrive(-turn_spd, turn_spd);
						if(gyro.getAngle() < cmd_start_angle + command[2]) rmFromPath(now);
					}
					break;
					
				case (int) move_enc:
					drive.tankDrive(spd, spd);
					if(enc.getRaw() - cmd_start_enc > command[2] / conv_count_to_in) rmFromPath(now);
					break;
					
				case (int) shoot:
					if(command[2] == shoot_high) {
						shootL.tankDrive(-high_shoot_spd, -high_shoot_spd);
						shootX.tankDrive(-high_shoot_spd, -high_shoot_spd);
					}
					else {
						shootL.tankDrive(-low_shoot_spd, -low_shoot_spd);
						shootX.tankDrive(-low_shoot_spd, -low_shoot_spd);
					}
					if(now - cmd_start_time > shoot_time) rmFromPath(now);
					break;
					
				case (int) reverse:
					drive.tankDrive(-spd, -spd);
					if(now - cmd_start_time > command[2] * in_per_millis) rmFromPath(now);
					break;

				case (int) move_enc_gyro:
					adj = (gyro.getAngle() - cmd_start_angle)/180;
					drive.tankDrive(
						constrain(spd - adj),
						constrain(spd + adj));
					if(enc.getRaw() - cmd_start_enc > command[2] / conv_count_to_in) rmFromPath(now);
					break;
				case (int) move_time_gyro:
					adj = (gyro.getAngle() - cmd_start_angle)/180;
					drive.tankDrive(
						constrain(spd - adj),
						constrain(spd + adj));
					if(now - cmd_start_time > command[2] * in_per_millis) rmFromPath(now);
					break;
					
				case (int) move_time:
					drive.tankDrive(spd, spd);
					if(now - cmd_start_time > command[2] * in_per_millis) rmFromPath(now);
					break;
					
			}
		}
		return this.toString();
	}
	
	protected static double constrain(double num, double max, double min) {
		return (num > max) ? 1 : (num < min) ? 0 : num;
	}
	
	static double constrain(double num) {
		return constrain(num, 1, 0);
	}
	
	private void rmFromPath(long now) {
		path.remove(0);
		reset_count(now);
	}
	
	private void reset_count(long now) {
//		SmartDashboard.putString("test", SmartDashboard.getString("test", "reset count: ") + ", " + now);
		cmd_start_time = now;
		cmd_start_angle = gyro.getAngle();
		cmd_start_enc = enc.getRaw();
	}
	
	public String toString() {
		String str = "";
		for(long[] item:path) {
			for(long val:item)
				str += val + ", ";
			str += "... ";
		}
		str += "time: " + (System.currentTimeMillis() - cmd_start_time) +
				" ... angle: " + (gyro.getAngle() - cmd_start_angle) +
				" ... encoder: " + (enc.getRaw() - cmd_start_enc);
		return str;
	}
}
