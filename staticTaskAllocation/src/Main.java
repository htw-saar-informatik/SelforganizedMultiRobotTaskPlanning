import java.awt.Point;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

public class Main {
	static ArrayList<Robot> robots = new ArrayList<Robot>();
	static ArrayList<TransportTask> tasks = new ArrayList<TransportTask>();
	
	public static void main(String[] args) throws Exception {	
		System.out.println(System.getProperty("user.dir"));
		RAI algo = new RAI();
		String testname ="";
		int roundsRAI = 10;
		int roundsOptimization = 100;
		if( args.length == 4) {
			if( args[0].equals("0") ){
				algo.setCostPerformance(RAI.COST_PERFORMANCE.DISTANCE);
				System.out.println("Use Distance");
			}else {
				algo.setCostPerformance(RAI.COST_PERFORMANCE.DRIVING_TIME);
				System.out.println("Use Driving Time");
			}
			testname = args[1];
			System.out.println("Use testcase: " + testname);
			roundsOptimization = Integer.parseInt(args[2]);
			System.out.println("RoundsOptimization: " + roundsOptimization);
			algo.setRoundsOptimization(roundsOptimization);
			roundsRAI = Integer.parseInt(args[3]);
			System.out.println("RoundsRAI: " + roundsRAI);
			
		}else {
			System.out.println("Usage 0(Distance)|1(Executiontime) testcasename roundsOptimization roundsRAI");
			System.exit(1);
		}
		readFile(testname);
		
		for (TransportTask t: tasks) {
			System.out.println(t.getName() + " [x=" + t.getPickUpPoint().getX() + ",y=" + t.getPickUpPoint().getY() + "]" + " -> [x=" + t.getDeliveryPoint().getX() + ",y=" + t.getDeliveryPoint().getY() + "]");
		}
		for (Robot r: robots) {
			System.out.println(r.getName() + " [x=" + r.getPosition().getX() + ",y=" + r.getPosition().getY() + "]");
		}
		
		for (TransportTask t: tasks) {
			algo.addTransportTask(t);
		}
		for (Robot r: robots) {
			algo.addRobot(r);
		}
		
		ArrayList<RAIResult> results = new ArrayList<>();
		for ( int i = 0; i < roundsRAI; i++) {
			
			long startTime = System.currentTimeMillis();
			algo.createDistancMatrix();
			RAIResult result = algo.startAlgo();
			System.out.println("DeadHeadDistance: " + result.deadHeadDistance);
			System.out.println("ExecutionTime: " + result.executionTime);
			long endTime = System.currentTimeMillis();
		    result.calculationTime  = endTime-startTime;
		    System.out.println("Calculation time: "+  result.calculationTime + " ms");
		    results.add(result);
		    System.out.println("Done round " + (i+1) + "/" + roundsRAI);
		}
		
		long averageTime = 0;
		double averageDeadHeadDistance = 0;
		double averageExecutionTime = 0;
		for( int i = 0; i < results.size(); i++) {
			RAIResult element = results.get(i);
			averageDeadHeadDistance+= element.deadHeadDistance; 
			averageExecutionTime+= element.executionTime;
			averageTime+= element.calculationTime;
		}
		System.out.println("");
		System.out.println("Result for " + roundsRAI + " rounds:");
		System.out.println("averageTime: " + averageTime/results.size());
		System.out.println("averageDeadHeadDistance: " + averageDeadHeadDistance/results.size());
		System.out.println("averageExecutionTime: " + averageExecutionTime/results.size());
		
	}
	
	public static void readFile(String testname) throws IOException {
		boolean startFound = false;
		int robotCounter = 0;
		int taskCounter = 0;
		boolean testCaseFound = false;
		try(BufferedReader br = new BufferedReader(new FileReader("./testfile"))) {
		    for(String line; (line = br.readLine()) != null; ) {
//		        System.out.println(line);
		        
		        if ( line.isEmpty() ) {
		        	if ( startFound ) {
		        		startFound = false;
		        	}
		        }else {
		        	String[] splitted = line.split(" ");
		        	if( splitted[0].equals("-") ) {
		        		if ( splitted[1].equals(testname)) {
		        			startFound = true;
		        			testCaseFound = true;
		        		}
		        	}else {
		        		if ( startFound ) {
		        			if ( splitted[0].equals("w") ) {
		        				//Ignore
		        			}else if ( splitted[0].equals("r") ) {
		        				Robot r = new Robot(new Point(Integer.parseInt(splitted[1]),Integer.parseInt(splitted[2])),"R" + robotCounter);
		        				robots.add(r);
		        				robotCounter++;
		        				
		        			}else {
		        				//Its a task
		        				Point p1 = new Point(Integer.parseInt(splitted[0]),Integer.parseInt(splitted[1]));
		        				Point p2 = new Point(Integer.parseInt(splitted[2]),Integer.parseInt(splitted[3]));
		        				TransportTask t = new TransportTask(p1, p2, "T"+taskCounter);
		        				taskCounter++;
		        				tasks.add(t);
		        			}
		        		}
		        	}
		        }
		        
		    }
		    
		}
		
		if(!testCaseFound) {
			System.out.println("Testcase not found");
			System.exit(3);
		}
	}
}
