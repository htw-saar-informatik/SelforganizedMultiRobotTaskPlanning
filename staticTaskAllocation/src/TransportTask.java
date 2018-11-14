import java.awt.Point;
import java.io.Serializable;

public class TransportTask implements Serializable{

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private Point pickupPoint;
	private Point deliveryPoint;
	private String name;
	
	public TransportTask(Point pickupPoint, Point deliveryPoint) {
		this.pickupPoint = pickupPoint;
		this.deliveryPoint = deliveryPoint;
	}
	
	public TransportTask(Point pickupPoint, Point deliveryPoint, String name) {
		this(pickupPoint,deliveryPoint);
		this.name = name;
	}
	
	public Point getPickUpPoint() {
		return pickupPoint;
	}
	
	public Point getDeliveryPoint() {
		return deliveryPoint;
	}
	
	public String getName() {
		return name;
	}
	
	/**
	 * Zeit, die benötigt wird um den Transportauftrag auszuführen.
	 * @return
	 */
	public int getTaskDuration() {
		double erg = (Math.abs(pickupPoint.x-deliveryPoint.x)+Math.abs(pickupPoint.y-deliveryPoint.y));
		return (int) erg;
	}
	
}
