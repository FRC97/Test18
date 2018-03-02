package org.usfirst.frc.team97.robot;

import java.util.ArrayList;
import java.util.List;

public class Auto {
	
	protected static final long
	delay = 0,
	move = 1,
	turn = 2,
	shoot = 3,
	reverse = 4;
	
	protected static final long cmp = 1, inc = 0; // complete, incomplete
	
	protected static final long shoot_low = 1, shoot_high = 2;
	
	// Auto distances -- inches
	protected static final long
	base_to_low = 168,
	base_to_high = 324,
	base_to_first_center = 0,
	base_to_second_mid = 0,
	first_center_to_first_side = 0,
	first_side_to_low = 0,
	low_to_fence = 0,
	second_ = 0,
	second_mid_to_center = 0;
	
	// [[completed, action type, action value][...]]
	List<long[]> path = new ArrayList<long[]>();
	private double spd;
	
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
			char center_force) {
		spd = auto_speed;
		addToPath(delay, auto_delay);
		
		//Center
		if(start_pos == 'C') {
			addToPath(move, base_to_first_center);
			char dir = center_force == 'C' ? low : center_force;
			turnOut(dir);
			addToPath(move, first_center_to_first_side);
			turnIn(dir);
			addToPath(move, first_side_to_low);
			if(low == dir) { // We're going towards the low goal -> shoot
				turnOut(dir);
				addToPath(reverse, low_to_fence);
				addToPath(shoot, shoot_low);
			}
		}
		
		// Right/Left
		else {			
			// If high is on our side and either there's no low or center can low -> High goal
			if(start_pos == high && (start_pos != low || center_capable < 0))
			{
				addToPath(move, base_to_high);
				turnOut(start_pos);
				addToPath(shoot, shoot_high);
			}
			
			// If low is open and center cannot low (and not going high) -> Low goal
			else if(start_pos == low && center_capable >= 0) {
				if(center_capable > 0) addToPath(delay, center_capable);
				addToPath(move, base_to_low);
				turnOut(start_pos);
				addToPath(reverse, low_to_fence);
				addToPath(shoot, shoot_low);
			}
			
			// None or low only and center can low -> Center middle (get out of the way)
			else {
				addToPath(move, base_to_second_mid);
				turnIn(start_pos);
				addToPath(move, second_mid_to_center);
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
}
