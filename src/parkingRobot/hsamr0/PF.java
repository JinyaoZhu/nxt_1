package parkingRobot.hsamr0;


public class PF{
	static final byte  map[] = new byte[25*200]; // map size = 200x200 (bit map)
	
	static final int MAX_MAP_X = 200;
	static final int MAX_MAP_Y = 200;
	static final int MIN_MAP_XY = 1;

	static final int NUM_OF_PARTICLES = 20;
	
	static float particles_x[] 	= new float[NUM_OF_PARTICLES];
	static float particles_y[]  = new float[NUM_OF_PARTICLES];
	static float particles_theta[]  = new float[NUM_OF_PARTICLES];
	static float particles_w[]  = new float[NUM_OF_PARTICLES];
	
	// predicted ir measurements for every particle
	static float predicted_ir_meas[] = new float[4];
	
	// IR sensor pose in robot coordinate 
	static final float IR_1_B_POSE[] = {0,0,0}; 
	static final float IR_2_B_POSE[] = {0,0,(float)Math.PI/2.0f}; 
	static final float IR_3_B_POSE[] = {0,0,(float)Math.PI}; 
	static final float IR_4_B_POSE[] = {0,0,-(float)Math.PI/2.0f}; 
	
	// estimated pose
	static float Ex[] = new float[3];
	
	/**
	 * robot specific constant: radius of left wheel
	 */
	static final float LEFT_WHEEL_RADIUS	= 	0.027f; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final float RIGHT_WHEEL_RADIUS	= 	0.027f; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final float WHEEL_DISTANCE		= 	0.120f; // only rough guess, to be measured exactly and maybe refined by experiments

	
	public PF() {
		
	}
	/*
	 * Description:
	 */
	public void initializeParticles() {
		for(int i=0; i<NUM_OF_PARTICLES; i++) {
			particles_x[i] = (float) (10.0 + 5.0*2.0*(Math.random()-0.5));
			particles_y[i] = (float) (10.0 + 5.0*2.0*(Math.random()-0.5));
			particles_theta[i] = (float) (0 + 0.5*2.0*(Math.random()-0.5));
			particles_w[i] = 1.0f/NUM_OF_PARTICLES;
		}
	}
	/*
	 * Description:
	 */
	public synchronized float get_scan_dist(float x0, float y0, float theta) {
		float x = x0;
		float y = y0;
		float theta_deg = theta*180.0f/(float)Math.PI;
		float tan_theta = (float)Math.tan(theta);
		float inv_tan_theta = 1.0f/tan_theta;
		
		float dx,dy;
		
		while(true) {
			if((x > MAX_MAP_X)||(y > MAX_MAP_Y)||(x < MIN_MAP_XY)||(y < MIN_MAP_XY))
				break;
			if(getMapPixel((int)(x+0.5f),(int)(y+0.5f)) == true)
				break;
			if ((theta_deg >= -45.0f) && (theta_deg < 45.0f)) {
			        y = y + tan_theta;
			        x = x + 1;
			}
			else if ((theta_deg >= 45.0f) && (theta_deg < 135.0f)) {
			        x = x + inv_tan_theta;
			        y = y + 1;
			}
			else if ((theta_deg >= -135.0f) && (theta_deg < -45.0f)) {
			        x = x - inv_tan_theta;
			        y = y - 1;
			}
			/* theta >=135 && theta < -135 */
			else {
			        y = y - tan_theta;
			        x = x - 1;
			}
		}
		
		dx = x - x0;
		dy = y - y0;
		
		return (float) Math.sqrt(dx*dx+dy*dy);
	}
	/*
	 * Description:
	 */
	private void predictMeasurements(float x, float y, float theta) {
		float ir_1_r_pose[] = new float[3];
		float ir_2_r_pose[] = new float[3];
		float ir_3_r_pose[] = new float[3];
		float ir_4_r_pose[] = new float[3];
		
		float robot_pose[] = {x,y,theta};
		
		for(int i=0; i<3; i++)
			ir_1_r_pose[i] = robot_pose[i] + IR_1_B_POSE[i];
		for(int i=0; i<3; i++)
			ir_2_r_pose[i] = robot_pose[i] + IR_2_B_POSE[i];
		for(int i=0; i<3; i++)
			ir_3_r_pose[i] = robot_pose[i] + IR_3_B_POSE[i];
		for(int i=0; i<3; i++)
			ir_4_r_pose[i] = robot_pose[i] + IR_4_B_POSE[i];
		
		predicted_ir_meas[0] = distanceToVoltageModell(get_scan_dist(ir_1_r_pose[0],ir_1_r_pose[1],ir_1_r_pose[2]));
		predicted_ir_meas[1] = distanceToVoltageModell(get_scan_dist(ir_2_r_pose[0],ir_2_r_pose[1],ir_2_r_pose[2]));
		predicted_ir_meas[2] = distanceToVoltageModell(get_scan_dist(ir_3_r_pose[0],ir_3_r_pose[1],ir_3_r_pose[2]));
		predicted_ir_meas[3] = distanceToVoltageModell(get_scan_dist(ir_4_r_pose[0],ir_4_r_pose[1],ir_4_r_pose[2]));
	}
	
	/*
	 * input : u1 = anglevelocity of right wheel
	 * 		   u2 = anglevelocity of left wheel
	 * */
	public void moveParticles(float u1_in, float u2_in, float dt) {
		
		float u1,u2;
		
		for(int i=0; i<NUM_OF_PARTICLES; i++) {
			u1 = u1_in;
			u2 = u2_in;
			float cos_theta = (float)Math.cos(particles_theta[i]);
			float sin_theta = (float)Math.sin(particles_theta[i]);
			particles_x[i] += (0.5f*LEFT_WHEEL_RADIUS*cos_theta*u1 + 0.5*RIGHT_WHEEL_RADIUS*cos_theta*u2)*dt;
			particles_y[i] += (0.5f*LEFT_WHEEL_RADIUS*sin_theta*u1 + 0.5*RIGHT_WHEEL_RADIUS*sin_theta*u2)*dt;
			particles_theta[i] += 0.5f*(LEFT_WHEEL_RADIUS+RIGHT_WHEEL_RADIUS)*(u1 - u2)*dt/WHEEL_DISTANCE;
			
			if (particles_x[i] > MAX_MAP_X)
				particles_x[i] = MAX_MAP_X;
			else if (particles_x[i] < MIN_MAP_XY)
				particles_x[i] = MIN_MAP_XY;
			
			if (particles_y[i] > MAX_MAP_Y)
				particles_y[i] = MAX_MAP_Y;
			else if (particles_y[i] < MIN_MAP_XY)
				particles_y[i] = MIN_MAP_XY;
			
			particles_theta[i] = warp_to_pi(particles_theta[i]);
		}
	}
	
	/*
	 * Description:
	 */
	public void calculateWeights(float z1, float z2, float z3, float z4) {
		
		for(int i=0; i<NUM_OF_PARTICLES; i++) {
			predictMeasurements(particles_x[i], particles_y[i], particles_theta[i]);
			float error_sq_sum = 0.0f;
			error_sq_sum += (z1-predicted_ir_meas[0])*(z1-predicted_ir_meas[0]);
			error_sq_sum += (z2-predicted_ir_meas[1])*(z2-predicted_ir_meas[1]);
			error_sq_sum += (z3-predicted_ir_meas[2])*(z3-predicted_ir_meas[2]);
			error_sq_sum += (z4-predicted_ir_meas[3])*(z4-predicted_ir_meas[3]);
			float error_norm = (float)Math.sqrt(error_sq_sum);
			if(error_norm < 0.1f)
				error_norm = 0.1f;
			particles_w[i] = 1/error_norm;
		}
		
		normalizeWeight();
	}
	
	/*
	 * Description:
	 */
	public void resample() {
		
	}
	
	/*
	 * Description:
	 */
	public void computeEx() {
		Ex[0] = 0;
		Ex[1] = 0;
		Ex[2] = 0;
		float sin_theta = 0;
		float cos_theta = 0;
		for(int i=0; i<NUM_OF_PARTICLES; i++) {
			Ex[0] += particles_x[i]*particles_w[i];
			Ex[1] += particles_y[i]*particles_w[i];
			sin_theta += Math.sin(particles_theta[i])*particles_w[i];
			cos_theta += Math.cos(particles_theta[i])*particles_w[i];
		}
		Ex[2] = (float) Math.atan2(sin_theta, cos_theta);
	}
	
	/*
	 * Description:
	 */
	public void updatePose(float u1, float u2, 
						float z1, float z2, float z3, float z4, 
						float dt) {
		moveParticles(u1,u2,dt);
		calculateWeights(z1,z2,z3,z4);
		computeEx();
		resample();
	}
	
	/*
	 * Description:
	 */
	private float warp_to_pi(float theta) {
		while(theta > Math.PI)
			theta -= 2*Math.PI;
		while(theta < -Math.PI)
			theta += 2*Math.PI;
		
		return theta;
	}
	
	/*
	 * Description:
	 */
	private boolean getMapPixel(int x, int y) {
		byte temp;
		temp = map[25*(y-1)+(x%25)/8];
		int shift = (x%8);
		temp = (byte) (temp>>shift);
		temp &= (byte)(0x01);
		if(temp == 0x01)
			return true;
		else
			return false;
	}
	/*
	 * Description:
	 */
	private float distanceToVoltageModell(float dist) {
		return dist;
	}
	
	/*
	 * Description:
	 */
	private void normalizeWeight() {
		float sum = 0;
		for(int i=0; i<NUM_OF_PARTICLES; i++) {
			sum += particles_w[i];
		}
		for(int i=0; i<NUM_OF_PARTICLES; i++) {
			particles_w[i] = particles_w[i]/sum;
		}
	}
}