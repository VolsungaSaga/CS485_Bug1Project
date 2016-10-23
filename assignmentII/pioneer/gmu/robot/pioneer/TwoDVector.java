package gmu.robot.pioneer;


public class TwoDVector {

	//A simple class that defines a vector in two dimensions. It's got a bunch of vector math functions, too.
	
	
	private double[] vector = new double[2];

	public TwoDVector(double x, double y){
		this.vector[0] = x;
		this.vector[1] = y;
	}
	
	
	public double getX(){ return this.vector[0];}
	public double getY(){ return this.vector[1];}
	
	//Given two vectors, performs vector addition and returns a new sum vector.
		
	public static TwoDVector vectorAdd(TwoDVector firstVector, TwoDVector secondVector){
		
		TwoDVector sumVector = new TwoDVector((firstVector.getX()+secondVector.getX()),
												(firstVector.getY() + secondVector.getY()));
		return sumVector;
	}
	
	
	
	//Returns a new, normalized version of the given vector. Non-destructive! 
	public static TwoDVector normalize(TwoDVector vectorObj){
		double newX = vectorObj.getX() / TwoDVector.magnitude(vectorObj);
		double newY = vectorObj.getY() / TwoDVector.magnitude(vectorObj);
		
		TwoDVector newVector = new TwoDVector(newX, newY);
		return newVector;
	}
	
	
	//Returns the Pythagorean sum of the given vector. 
	public static double magnitude(TwoDVector vector){
		
		return Math.sqrt((vector.getX() * vector.getX()) + (vector.getY() * vector.getY()));
		
	}
	
	
	
	
	
	
	
	
	
	
}
