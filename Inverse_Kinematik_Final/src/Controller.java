import java.util.ArrayList;

import controlP5.*;
import processing.core.*;
import processing.data.Table;
import processing.serial.*;

public class Controller extends PApplet {
	Main m;
	Command cmd;
	Quaternion Q;
	Sender sender;
	
	boolean running = true;
	ArrayList<KoordinatenSystem> p = new ArrayList<KoordinatenSystem>();
	KoordinatenSystem kstart;
	double[][] T_nozzle = {
			{1,0,0,200},
			{0,1,0,100},
			{0,0,1,600},
			{0,0,0,1}};
	Jama.Matrix T_Nozzle = new Jama.Matrix(T_nozzle);
	Table traj;
	
	long current_time;
	long last_time = System.nanoTime();
	
	public static void main(String[] args) {
		PApplet.main("Controller");
		//System.out.print(ceil(3));
	}

	public void settings() {
		size(1800, 1000, P3D);
		smooth(8);
		println("started");
	}

	public void setup() {
		m = new Main(this);
		cmd = new Command(this, m.ik, m);
		sender = new Sender(this);
		
		frameRate(40);
		//traj = loadTable("traj_ecke_xi1.csv");
		p.add(new KoordinatenSystem(this, 0, 0, 0, 0, 0, 0));
		p.add(new KoordinatenSystem(this, 400, 0, 500, 0, 0, 0));
		p.add(new KoordinatenSystem(this, 400, 0, 500, 0, -181, 0));
		p.add(new KoordinatenSystem(this, 400, 0, 500, 90, 90, 90));
		p.add(new KoordinatenSystem(this, 400, 0, 500, 90, 0, 90));
		p.add(new KoordinatenSystem(this, 400, 0, 50, 0, 180, 0));
		p.add(new KoordinatenSystem(this, 200, 0, 50, 0, 180, 0));
		p.add(new KoordinatenSystem(this, 300, 100, 50, 0, 180, 0));
		p.add(new KoordinatenSystem(this, 150, 400, 400, 0, 90, 0));
		p.add(new KoordinatenSystem(this, 150, -400, 400, 0, 90, 0));
		//p.add(new KoordinatenSystem(this, 150, -300, 400, 0, 90, 40));
		//p.add(new KoordinatenSystem(this, 450, -100, 350, 30, 70, 0));
		//p.add(new KoordinatenSystem(this, 373, 50, 200, 0, 180, 0));
		//p.add(new KoordinatenSystem(this, 373, 50, 200, 0, 180, 0));
		m.set_Pointlist(p);
		thread("commands");
		thread("dataTransfer");

	}
	public void dataTransfer() {
		while(true) {
			sender.sendData(-(float)m.ik.q_dot.get(0, 0),-(float)m.ik.q_dot.get(1,0),(float)m.ik.q_dot.get(2,0),-(float)m.ik.q_dot.get(3,0),-(float)m.ik.q_dot.get(4,0),(float)m.ik.q_dot.get(5,0));
			//println(degrees((float)m.ik.q_dot.get(0, 0))-sender.rq[0]);
			sender.readData();
			delay(50);
		}
		
	}
	public void commands() {

		while(running) {

		cmd.waitTime(1,1);
		//cmd.addStatesToList();
		cmd.linear_Path(p.get(1), 2, 2);
		cmd.waitTime(1, 3);
		cmd.linear_Path(p.get(2), 1, 4);
		cmd.linear_Path(p.get(3), 2, 5);
		cmd.linear_Path(p.get(4), 2, 6);
		cmd.linear_Path(p.get(5), 2, 7);
		cmd.linear_Path(p.get(6), 2, 8);
		cmd.circle_Path(p.get(7),p.get(5), 3, 9);
		cmd.linear_Path(p.get(8), 2, 10);
		cmd.linear_Path(p.get(9), 3, 11);
		cmd.move_Home(10, -1);
		cmd.move_Rest(10, -3);
		//cmd.circle_Path(p.get(4), p.get(3), 6, 6);
		//cmd.linear_Path(p.get(2), 6, 7);
		//cmd.circle_Path(p.get(4), p.get(3), 6, 8);
		//cmd.linear_Path(p.get(2), 6, 9);
		//cmd.circle_Path(p.get(4), p.get(3), 6, 10);
		//cmd.linear_Path(p.get(2), 6, 11);
		//cmd.circle_Path(p.get(4), p.get(3), 6, 12);
		//cmd.moveToPrintStart(traj, T_Nozzle, 2, 2);
		//cmd.doPrint2(traj,T_Nozzle, 3);
		//cmd.saveList(4);
		//m.setQ(Q);
//		current_time = System.nanoTime();
//		double dt = 0.000000001 * (current_time - last_time);
//		last_time = current_time;
		//println(dt);
		

		//delay(5);
		}
	}

	public void draw() {
		m.display();
	}

	public void controlEvent(ControlEvent theEvent) {
		
		if (theEvent.isController()) {

			switch (theEvent.getName()) {

			case "Reset Cam":

				m.setCam();
				break;
				
			case "Move Home":
				cmd.counter_global = -1;
				cmd.running = false;
				break;
				
			case "Move Rest":
				cmd.counter_global = -3;
				cmd.running = false;
				break;
				
			case "Exit":

				exit();
				break;

			case "saveFrame":
				
				println("save");
				saveFrame("RobotPic-####.png");
				break;

			case "Create Traj":

				println("create Traj");
				break;

			case "Simulate Traj":

				println("Simulate Traj");
				break;

			case "Run Traj":

				println("Run Traj");
				cmd.counter_global = 1;
				cmd.running = false;
				break;

			case "Show Points":
				if(m.show_points) {
					m.show_points = false;
				}else {
					m.show_points = true;
				}
			}
		}
	}
}
