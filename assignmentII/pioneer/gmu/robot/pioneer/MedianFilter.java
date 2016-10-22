package gmu.robot.pioneer;

import java.util.Arrays;

public class MedianFilter {

	PioneerRobot robot;
	
	MedianFilter(PioneerRobot robot) {
		this.robot = robot;
	}
	
	public double[] getFilteredSonarValues() {
				int i = 0;
		double[][] store = new double[16][5];
		
		// window size is 5
		while (i < 16)
		{	
			for (int j = 0; j < 5; ++j) {
				store[i][j] = robot.getSonar(i);
			}
			++i;
		}
		
		// sort store
		for (int m = 0; m < 5; ++m) {
			Arrays.sort(store[m]);
		}
		
		double[] ret = new double[16];
		for (int n = 0; n < 16; ++n) {
			ret[n] = store[n][2]; 
		}
		return ret;
	}
}
