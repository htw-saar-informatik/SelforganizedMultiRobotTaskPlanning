import java.awt.Point;
import java.io.Serializable;

public class Robot implements Serializable{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	public Point position;
	public String name;
	
	public Robot(Point position, String name) {
		this.position = position;
		this.name = name;
	}
	
	public Point getPosition() {
		return this.position;
	}

	public String getName() {
		return this.name;
	}
}
