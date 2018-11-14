import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

public class RAI {

	private List<Robot> robots;
	private List<TransportTask> transportTasks;
	private int[][] distanceMatrix;
	public COST_PERFORMANCE costPerformance;
	boolean showLogs = false;
	private int roundsOptimization = 100;

	public RAI() {
		robots = new ArrayList<Robot>();
		transportTasks = new ArrayList<TransportTask>();
		costPerformance = COST_PERFORMANCE.DISTANCE;
	}

	public void setRoundsOptimization(int rounds) {
		roundsOptimization = rounds;
	}

	public void addRobot(Robot r) {
		robots.add(r);
	}

	public void addTransportTask(TransportTask task) {
		transportTasks.add(task);
	}

	public static enum COST_PERFORMANCE {
		DISTANCE, DRIVING_TIME
	}

	public void setCostPerformance(COST_PERFORMANCE cp) {
		costPerformance = cp;
	}

	/**
	 * Creates the distance matrix
	 */
	public void createDistancMatrix() {
		int length = robots.size() + transportTasks.size();
		distanceMatrix = new int[length][length];

		// calculate distance
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				// System.out.println(i + " " + j);
				distanceMatrix[i][j] = calculateDistance(i, j);
			}
		}

		System.out.println("Matrix:");
		printDistanceMatrix();
		int lowerBoundDistance = calculateLowerBoundDistance();
		System.out.println("Min. Distance: " + lowerBoundDistance);
		int lowerBoundExectionTime = calculateLowerBoundExecutionTime();
		System.out.println("Min execution Time: " + lowerBoundExectionTime);
	}

	/**
	 * Calculates the min. distanz as lower limit
	 * 
	 * @return
	 */
	private int calculateLowerBoundDistance() {
		int minValue = 0;
		int value;
		int lowerBound = 0;
		for (int i = 0; i < transportTasks.size(); i++) {
			minValue = -1;
			// search min distance to drive to each task
			for (int j = 0; j < (robots.size() + transportTasks.size() - 1); j++) {
				// search lowest value in column
				value = distanceMatrix[j][i + robots.size()];
				if (value != -1) { // Task to itself
					if (minValue == -1) {
						minValue = value;
					} else {
						if (value < minValue) {
							minValue = value;
						}
					}
				}
			}
			lowerBound += minValue;
		}
		return lowerBound;
	}

	private int calculateLowerBoundExecutionTime() {

		// ArrayList<Integer> minValues = new ArrayList<>();
		PriorityQueue<Integer> minValues = new PriorityQueue<>();

		int minValue = 0;
		int globalMinValue = -1;
		int value;
		TransportTask task;
		for (int i = 0; i < transportTasks.size(); i++) {
			minValue = -1;
			task = transportTasks.get(i);
			for (int j = 0; j < (robots.size() + transportTasks.size() - 1); j++) {
				value = distanceMatrix[j][i + robots.size()];
				value += task.getTaskDuration();
				if (value != -1) {
					if (minValue == -1) {
						minValue = value;
					} else {
						if (distanceMatrix[j][i + robots.size()] != -1) {
							if (value < minValue) {
								minValue = value;
							}
						}
					}
				}
			}
			// minValue = minValue+task.getTaskDuration();
			minValues.add(minValue);
			if (globalMinValue == -1) {
				globalMinValue = minValue;
			} else {
				if (globalMinValue > minValue) {
					globalMinValue = minValue;
				}
			}
		}

		// Is the number of tasks that at least one robots has to do together
		int minTaskCount = (int) Math.ceil((double) transportTasks.size() / (double) robots.size());

		// Take the minTaskCount values
		int lowestSum = 0;
		for (int i = 0; i < minTaskCount; i++) {
			lowestSum += minValues.poll();
		}

		return globalMinValue > lowestSum ? globalMinValue : lowestSum;
	}

	private void printDistanceMatrix() {
		int length = distanceMatrix.length;
		String line;
		for (int i = 0; i < length; i++) {
			line = "";
			for (int j = 0; j < length; j++) {
				line += distanceMatrix[i][j] + " ";
			}
			System.out.println(line);
		}
	}

	public RAIResult startAlgo() {
		int length = robots.size() + transportTasks.size();
		ArrayList<Integer> s = new ArrayList<>();

		// Start with random Node
		int rnd = getRandomNumberInRange(0, length - 1);
		s.add(rnd);
		s.add(rnd);

		while (s.size() != (length + 1)) {
			// Get next vertex
			int nextVertex = getVertexNotOnTour(s, length);

			// Insert Vertex in most cheap way in tour
			s = insertCheap(s, nextVertex);
		}
		System.out.println("First Tour: \n");
		printTour(s);

		System.out.println("Start optimization");
		ArrayList<Integer> optimizedTour = optimizeTour(s);
		System.out.println("Optimization done");
		System.out.println("Final Tour:");
		printTour(optimizedTour);

		RAIResult result = new RAIResult();
		result.deadHeadDistance = getTourDistance(optimizedTour);
		result.executionTime = getTourExecutionTime(optimizedTour);

		return result;
	}

	private ArrayList<Integer> optimizeTour(ArrayList<Integer> originalTour) {
		// 1) Choose random numbers i,j 0..(robots.size()+transportTasks.size())-1, i <=
		// j
		int max = robots.size() + transportTasks.size() - 1;
		int length = robots.size() + transportTasks.size();
		int originalTourCosts = getCostsForTour(originalTour);

		for (int counter = 0; counter < roundsOptimization; counter++) {
			print("Runde: " + counter + " Aktuelle Tour:");

			// printTour(originalTour);
			// System.out.println("");
			// System.out.println(originalTour);

			// copy tour
			ArrayList<Integer> currentTour = new ArrayList<>();
			for (int e : originalTour) {
				currentTour.add(e);
			}

			int i = getRandomNumberInRange(0, max);
			int j = getRandomNumberInRange(0, max);

			// swap if i > j
			if (i > j) {
				int tmp = i;
				i = j;
				j = tmp;
			}
			currentTour.subList(i, j + 1).clear();

			// connect vertex i-1 with vertex j + 1
			// if i = 0 und j = max, complete route deleted, delete last element
			if (i == 0 && j == max) {
				currentTour.clear();
			} else {
				// i== 0: first node was deleted, j == max: last node was deleted
				if (i == 0) {
					// delete last element
					currentTour.remove(currentTour.size() - 1);
					// Add new first node at the end of the tour to complete the tour
					currentTour.add(currentTour.get(0));
				}
			}

			// complete currentTour
			while (currentTour.size() != (length + 1)) {
				// Get next vertex
				int nextVertex = getVertexNotOnTour(currentTour, length);

				// Insert Vertex in most cheap way in tour
				currentTour = insertCheap(currentTour, nextVertex);
				checkIfTourIsValid(currentTour);
				// System.out.println(currentTour);
			}

			print("New Tour:");
			// printTour(currentTour);
			// print(currentTour);

			// compare new subPath with original Tour
			int currentTourCosts = getCostsForTour(currentTour);
			if (currentTourCosts < originalTourCosts) {
				originalTour = currentTour;
				originalTourCosts = currentTourCosts;
				System.out.println("Found Tour with lower cost (Round " + counter + "):");
				printTour(currentTour);
			} else {
				// System.out.println("old solution is better");
			}

		}
		return originalTour;
	}

	private void checkIfTourIsValid(ArrayList<Integer> tour) {
		ArrayList<Integer> tmp = new ArrayList<>();
		int counter = 0;
		for (int value : tour) {
			if (tmp.contains(value)) {
				if ((tmp.indexOf(value) == 0) && (counter == tour.size() - 1)) {

				} else {
					System.out.println("Not a valid Tour!");
				}

			} else {
				tmp.add(value);
			}
			counter++;
		}
	}

	private void printTour(ArrayList<Integer> s) {
		String ergebnis = "";
		for (int entry : s) {
			if (entry < robots.size()) {
				ergebnis += robots.get(entry).getName() + " ";
			} else {
				ergebnis += transportTasks.get(entry - robots.size()).getName() + " ";
			}
		}
		System.out.println(ergebnis + " (" + getCostsForTour(s) + ")");
	}

	/*
	 * insert the new node in the tour on position with lowest cost
	 */
	private ArrayList<Integer> insertCheap(ArrayList<Integer> tour, int element) {
		// generate tour.size-1 possible subtours

		// ArrayList<ArrayList<Integer>> list = new ArrayList<>();
		ArrayList<Integer> bestSolution = new ArrayList<>();
		int minCosts = -1;

		// case 1: tour is empty, make tour valid
		if (tour.size() == 0) {
			// System.out.println("Tour was empty!");
			bestSolution.add(element);
			bestSolution.add(element);
			return bestSolution;
		}

		// case 2: tour has 1 element, make tour valid
		if (tour.size() == 1) {
			System.out.println("Tour was not closed!");
			tour.add(element);
		}
		int rounds = tour.size() - 1;

		for (int i = 0; i < (rounds); i++) {
			if (i > 0) {
				tour.remove(i);
			}
			tour.add(i + 1, element);
			if (minCosts == -1) {
				bestSolution = tour;
				minCosts = getCostsForTour(tour);
			} else {
				int tmpDistance = getCostsForTour(tour);
				if (tmpDistance < minCosts) {
					bestSolution = tour;
					minCosts = tmpDistance;
				}
			}
		}
		return bestSolution;
	}

	/**
	 * returns the total deadhead distance to drive the tour
	 * 
	 * @param tour
	 * @return
	 */
	private int getTourDistance(ArrayList<Integer> tour) {
		int totalDistance = 0;
		for (int i = 0; i < (tour.size() - 1); i++) {
			totalDistance += distanceMatrix[tour.get(i)][tour.get(i + 1)];
		}
		return totalDistance;
	}

	private int getCostsForTour(ArrayList<Integer> tour) {
		switch (costPerformance) {
		case DISTANCE:
			return getTourDistance(tour);
		case DRIVING_TIME:
			return getTourExecutionTime(tour);
		default:
			return 0;
		}
	}

	/**
	 * Returns the total execution time for this tour
	 * 
	 * @param tour
	 * @return
	 */
	private int getTourExecutionTime(ArrayList<Integer> tour) {
		int maxExecutionTime = 0;
		int currentExecutionTime = 0;
		int executionTimeBeginning = 0;
		boolean hasSetExecutionTimeBeginning = false;
		int firstValue = tour.get(0);
		boolean firstPositionIsRobot = false;
		int lastRobot;
		// chek if first element is a robot. If not, the first elements have to be added
		// to the last robot found in the tour
		firstPositionIsRobot = checkIsRobot(tour.get(0));
		int currentValue;
		int nextValue;
		for (int i = 0; i < tour.size() - 2; i++) {
			currentValue = tour.get(i);
			nextValue = tour.get(i + 1);
			if (checkIsRobot(currentValue)) {

				// Check if tour starts with a robot. if not, if this is the first robot in the
				// tour, the execution time to this element has to be saved
				// and later added to the last robot
				if (!checkIsRobot(firstValue)) {
					if (!hasSetExecutionTimeBeginning) {
						executionTimeBeginning = currentExecutionTime;
						hasSetExecutionTimeBeginning = true;
					}
				}

				// check if current execution time > max. execution time
				if (currentExecutionTime > maxExecutionTime) {
					maxExecutionTime = currentExecutionTime;
				}

				currentExecutionTime = 0;
				lastRobot = currentValue;

				// calculate driving time from current robot to next transport task
				currentExecutionTime += distanceMatrix[currentValue][nextValue];
			} else {
				// execution time for the current transporttask
				currentExecutionTime += transportTasks.get(currentValue - robots.size()).getTaskDuration();

				// driving time from the current task to the next element (task|robot)
				currentExecutionTime += distanceMatrix[currentValue][nextValue];
			}
		}

		// check element before last. If it is not a robot, the distance from the task
		// has to be added
		// if it is a robot, the current time has to be checked
		int secondToLastValue = tour.get(tour.size() - 2);
		if (checkIsRobot(secondToLastValue)) {
			lastRobot = secondToLastValue;

			// Check if tour starts with a robot. if not, if this is the first robot in the
			// tour, the execution time to this element has to be saved
			// and later added to the last robot
			if (!checkIsRobot(firstValue)) {
				if (!hasSetExecutionTimeBeginning) {
					executionTimeBeginning = currentExecutionTime;
					hasSetExecutionTimeBeginning = true;
				}
			}

			// check if current execution time > max. execution time
			if (currentExecutionTime > maxExecutionTime) {
				maxExecutionTime = currentExecutionTime;
			}
			currentExecutionTime = 0;

		} else {
			currentExecutionTime += transportTasks.get(secondToLastValue - robots.size()).getTaskDuration();

			// check if current execution time > max. execution time
			if (currentExecutionTime > maxExecutionTime) {
				maxExecutionTime = currentExecutionTime;
			}
		}

		// check if the route starts with an robot. If not, the execution time for
		// starting tasks has to be added the the last robot
		if (!checkIsRobot(firstValue)) {
			if (executionTimeBeginning > 0) {
				currentExecutionTime += executionTimeBeginning;
				currentExecutionTime += distanceMatrix[tour.get(tour.size() - 2)][tour.get(tour.size() - 1)];

				if (currentExecutionTime > maxExecutionTime) {
					maxExecutionTime = currentExecutionTime;
				}

			}
		}
		return maxExecutionTime;
	}

	private boolean checkIsRobot(int index) {
		return index < robots.size();
	}

	/**
	 * returns a vertex that has not been added to the tour yet
	 * 
	 * @param tour
	 * @param length
	 * @return
	 */
	private int getVertexNotOnTour(ArrayList<Integer> tour, int length) {
		if (tour.size() == length + 1) {
			System.out.println("no elements left");
		}
		boolean found = false;
		int rnd;
		do {
			found = false;
			rnd = getRandomNumberInRange(0, length - 1);
			// Check if number is already used
			for (int i = 0; i < tour.size(); i++) {
				if (tour.get(i) == rnd) {
					found = true;
				}
			}
		} while (found);
		return rnd;
	}

	/**
	 * calculate distance between two elements (tasks|robots)
	 * 
	 * @param firstIndex
	 * @param secondIndex
	 * @return
	 */
	private int calculateDistance(int firstIndex, int secondIndex) {
		if (firstIndex == secondIndex) {
			return -1;
		}

		// check if elements are robots or tasks

		// Robot -> Robot
		if (firstIndex < robots.size() && secondIndex < robots.size()) {
			return 0;
		}

		// Robot -> Task
		if (firstIndex < robots.size() && secondIndex >= robots.size()) {
			Robot r1 = robots.get(firstIndex);
			TransportTask t1 = transportTasks.get(secondIndex - robots.size());
			// manhatten distance
			double erg = (Math.abs(r1.getPosition().x - t1.getPickUpPoint().x)
					+ Math.abs(r1.getPosition().y - t1.getPickUpPoint().y));
			return (int) erg;
		}

		// Task -> Task
		if (firstIndex >= robots.size() && secondIndex >= robots.size()) {
			TransportTask t1 = transportTasks.get(firstIndex - robots.size());
			TransportTask t2 = transportTasks.get(secondIndex - robots.size());
			// manhatten distance
			double erg = (Math.abs(t1.getDeliveryPoint().x - t2.getPickUpPoint().x)
					+ Math.abs(t1.getDeliveryPoint().y - t2.getPickUpPoint().y));
			return (int) erg;
		}

		// Task -> Robot
		if (firstIndex >= robots.size() && secondIndex < robots.size()) {
			return 0;
		}
		return 0;
	}

	private static int getRandomNumberInRange(int min, int max) {

		if (min >= max) {
			throw new IllegalArgumentException("max must be greater than min");
		}

		return (int) (Math.random() * ((max - min) + 1)) + min;
	}

	private void print(String s) {
		if (showLogs) {
			System.out.println(s);
		}
	}

	private void print(ArrayList<Integer> list) {
		if (showLogs) {
			System.out.println(list);
		}
	}

}
