import java.util.ArrayList;

import Jama.Matrix;
import controlP5.*;
import processing.core.*;
import processing.data.Table;
import processing.data.TableRow;

public class Controller extends PApplet {
	Main m;
	Table path;
	Trajectory trajectory;
	long current_time = 0;
	long last_time = 0;
	double time = 0;
	int step = 1;

	public static void main(String[] args) {
		PApplet.main("Controller");
		System.out.print(ceil(3));
	}

	public void settings() {
		size(1800, 1000, P3D);

		smooth(8);
	}

	public void setup() {
		m = new Main(this);
		frameRate(1000);
		// traj = loadTable("test_traj_angepasst.csv");
		path = loadTable("schnecke_xi1.csv");
		trajectory = new Trajectory(this);
		trajectory.createTraj(path);
		thread("runTraj");

	}

	public void draw() {

		m.display();
		trajectory.show(step);
		m.gui();

	}

	public void runTraj() {
		while (true) {
			Table table = trajectory.getTraj();
			current_time = System.nanoTime();
			double dt = 0.000000001 * (current_time - last_time);
			//println(dt);
			last_time = current_time;
			time = time + dt;
			
			if (step < table.getRowCount()-1) {
				TableRow row1 = table.getRow(step);
				TableRow row2 = table.getRow(step+1);
				double ddt = row1.getDouble(12);
				if (time >= ddt) {
					double x1 = row1.getDouble(0);
					double y1 = row1.getDouble(1);
					double z1 = row1.getDouble(2);
					
					double xx1 = row1.getDouble(3);
					double xy1 = row1.getDouble(4);
					double xz1 = row1.getDouble(5);
					
					double yx1 = row1.getDouble(6);
					double yy1 = row1.getDouble(7);
					double yz1 = row1.getDouble(8);
					
					double zx1 = row1.getDouble(9);
					double zy1 = row1.getDouble(10);
					double zz1 = row1.getDouble(11);
					
					
					
					double x2 = row2.getDouble(0);
					double y2 = row2.getDouble(1);
					double z2 = row2.getDouble(2);
					
					double xx2 = row2.getDouble(3);
					double xy2 = row2.getDouble(4);
					double xz2 = row2.getDouble(5);
					
					double yx2 = row2.getDouble(6);
					double yy2 = row2.getDouble(7);
					double yz2 = row2.getDouble(8);
					
					double zx2 = row2.getDouble(9);
					double zy2 = row2.getDouble(10);
					double zz2 = row2.getDouble(11);
					
					double[][] r1 = {
							{xx1, yx1, zx1},
							{xy1, yy1, zy1},
							{xz1, yz1, zz1}};
					
					Jama.Matrix R1 = new Jama.Matrix(r1);
					
					double[][] r2 = {
							{xx2, yx2, zx2},
							{xy2, yy2, zy2},
							{xz2, yz2, zz2}};
					
					Jama.Matrix R2 = new Jama.Matrix(r2);
	
					R2.print(3, 3);
					Quaternion Q1 = new Quaternion(R1);
					Quaternion Q2 = new Quaternion(R2);
					Quaternion Q_dot = Q2.minus(Q1);
					Q_dot.times(2/time);
					Quaternion w = Q_dot.times(Q1.inverse());
					Jama.Matrix W = w.getVector();
					double omega = W.norm2();
					
					
					Jama.Matrix Re = R1.transpose().times(R2);
					
					Quaternion Qres = new Quaternion(Re);
					double QresLen = Qres.getVector().norm2();
					double theta = 2*Math.atan2(Qres.getVector().norm2(), Qres.n);
					println(Math.toDegrees(theta));
//					Jama.Matrix Axis2 = Qres.getVector();
//					if(QresLen>0.00000001) {
//					Axis2  = Axis2.times(1/QresLen);
//					//Axis2.times(1/QresLen);
//					}else {
//						double[][] res = {{1},{0},{0}};
//						Jama.Matrix Res = new Jama.Matrix(res);
//						Axis2 = Res;
//					}
//					
//					
//					Jama.Matrix Omega2 = Axis2.times(theta/time);
					
					double dx = x2-x1;
					double dy = y2-y1;
					double dz = z2-z1;
					
					double l = Math.sqrt(dx*dx+dy*dy+dz*dz);
					double v = l/time;
					

					
					step++;
					time = 0;
				}
			}
		}
	}

	public void controlEvent(ControlEvent theEvent) {
		println("pressed");
		if (theEvent.isController()) {

			switch (theEvent.getName()) {

			case "Reset Cam":

				m.setCam();
				break;

			case "Exit":

				exit();
				break;

			case "Test Path":

				println("Test Path");
				break;

			case "Create Traj":

				println("create Traj");
				break;

			case "Simulate Traj":

				println("Simulate Traj");
				break;

			case "Run Traj":

				println("Run Traj");
				break;

			case "Show Points":
				if (m.show_points) {
					m.show_points = false;
				} else {
					m.show_points = true;
				}
			}
		}
	}
}
