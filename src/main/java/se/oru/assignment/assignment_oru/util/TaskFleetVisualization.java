package se.oru.assignment.assignment_oru.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.util.FleetVisualization;

public interface TaskFleetVisualization extends FleetVisualization {
	

	public void displayTask(Pose start, Pose goal, int id, String color);

	void removeTask(int taskId);

}
