package dk.itu.mario.engine;


import dk.itu.mario.engine.PlayerProfile;
import dk.itu.mario.engine.level.Level;
import dk.itu.mario.engine.sprites.Enemy;

//This player profile wants to be constantly jumping (wants 1 jump per every 4 blocks)
public class CloudClimber extends PlayerProfile{
	
	//Return whether Mario can stand on a certain block.
	private static boolean isStandable(byte block)
	{
		if(block == Level.TUBE_SIDE_LEFT)
			return true;
		if(block == Level.HILL_TOP)
			return true;
		if(block == Level.ROCK)
			return true;
		if(block == Level.CANNON_BASE)
			return true;
		if(block == Level.CANNON_MID)
			return true;
		if(block == Level.CANNON_TOP)
			return true;
		return false;
	}
	
	
	//evaluation for this player profile
	//returns a number bounded from 0 to 1
	public double evaluateLevel(Level level)
	{
		// 14 lowest, 0 highest
		int[] highestPlatform = new int[50];
		
		for (int x = 0; x < highestPlatform.length; x++)
			highestPlatform[x] = 15;

		for (int x = 0; x<200; x++){
			int firstPlatform = 15;
			for (int y = 2; y<15; y++){
				if(isStandable(level.getBlock(x,y))){
					firstPlatform = y;
					break;
				}
			firstPlatform = -1;
			}
			
			int positionInArray = (int)x/4;
			if(firstPlatform < highestPlatform[positionInArray]){
				highestPlatform[positionInArray] = firstPlatform;
			}
		}
		

		double score = 0.0;
		double numTrues = 0.0;

		int lastValue = 2;
		
		for(int i = 0; i<highestPlatform.length; i++){
			int thisValue = highestPlatform[i];
			
			// 0.35 for delta: 0.5 for difference of 2 in highest platform height.
			// 0.2 for diff=1,3; 0.05 for diff=0,4; 0 otherwise.
			double deltaScore = Math.max(0, 0.35-Math.abs(thisValue-lastValue)*0.15);
		
			// 0.7 for absolute height: 0.5 for height=8.
			// 0.6 for height=7,9; 0.5 for height=6,10; etc.
			double heightScore = Math.max(0, 0.7-Math.abs(thisValue-8)*0.1);
			
			numTrues += deltaScore + heightScore;
			lastValue = thisValue;
		
		}

		score = numTrues/50.0;

		// Though max score is 1.05, score is capped at 1.
		if (score>1.0){
			score = 1.0;
		}

		return score;
	}
}