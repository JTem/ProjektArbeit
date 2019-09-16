import java.util.ArrayList;

import controlP5.*;
import processing.core.*;
import processing.data.Table;
import remixlab.proscene.*;

public class Main {

	PApplet p;
	PShape base, ax1, ax2, ax3, ax4, ax5, ax6, extr;
	Scene scene;

	ControlP5 cp5;
	Inverse_Kinematics ik;
	KoordinatenSystem ks, ke, show_Trans, k1, k2, k3, kQ;
	//PrintPath print;
	Jama.Matrix test_Trans;
	long last_time, current_time;
	Button ex;
	double t = 0;
	boolean enableGrid = true;
	boolean show_points = true;
	ArrayList<KoordinatenSystem> pointList = new ArrayList<KoordinatenSystem>();
	float mq1 = 0, mq2 = 0, mq3 = 0, mq4 = 0, mq5 = 0, mq6 = 0;
	Table traj;
	Command cmd;
	int step = 1;

	
	public Main(PApplet _p) {
		p = _p;
		cp5 = new ControlP5(p);
		scene = new Scene(p);

		ik = new Inverse_Kinematics(p);
		k1 = new KoordinatenSystem(p, 200, 100, 600, 0, 0, 0);
		k2 = new KoordinatenSystem(p, 282.3, -150, 416.5, 0, 0, 0);
		k3 = new KoordinatenSystem(p, 150.3, -75, 486.5, 0, 0, 0);
		ks = new KoordinatenSystem(p, 100, -500, 200, 0, 0, 0);
		ke = new KoordinatenSystem(p, 100, -300, 600, 0, 50, 0);
		kQ = new KoordinatenSystem(p, 0, 0, 0, 0, 0, 0);
		show_Trans = new KoordinatenSystem(p, 0, 0, 0, 0, 0, 0);
		//traj = p.loadTable("traj_ecke_xi1.csv");
		//print = new PrintPath(p, traj, 0, 0, 0);
		setCam();
		setupScene();
		loadParts();
		loadTextLabel();
		cp5.setAutoDraw(false);

	}

	public void display() {

		p.background(255);

		scene.drawAxes(215);
		setLights();

		mq1 = (float) ik.q1;
		mq2 = (float) ik.q2;
		mq3 = (float) ik.q3;
		mq4 = (float) ik.q4;
		mq5 = (float) ik.q5;
		mq6 = (float) ik.q6;
		//k1.show();
		ik.showKS(7);
		//kQ.show();

		if (show_points)
			showPoints();

		drawGrid();
		drawRobot();
		gui();

	}

	void setQ(Quaternion Q) {
		kQ.setTrans(Q.getTransformationmatrix());

	}

	void set_Pointlist(ArrayList<KoordinatenSystem> p) {
		pointList = p;
	}

	void setProgression(int _step) {
		step = _step;
	}

	void showPoints() {
		for (KoordinatenSystem p : pointList) {
			p.show();
		}
	}

	void drawRobot() {

		p.fill(255, 165, 0);
		p.noStroke();
		p.pushMatrix();
		p.scale(0.085106382978723f);
		p.shape(base);

		p.popMatrix();
		p.pushMatrix();
		p.translate(200,100,600);
		p.shape(extr);
		p.popMatrix();
		
		p.rotateZ(p.HALF_PI);
		p.rotateZ(mq1);
		p.translate(0, -90, 159);
		p.pushMatrix();
		p.scale(0.130767272727272f);
		p.shape(ax1);
		p.popMatrix();

		p.rotateX(-mq2);
		p.translate(-25.5f, 0, 300);
		p.pushMatrix();
		p.scale(0.22856622857143f);
		p.shape(ax2);
		p.popMatrix();

		p.rotateX(-mq3);
		p.translate(25.5f, -15, 57);
		p.pushMatrix();
		p.scale(0.10477272727273f);
		p.shape(ax3);
		p.popMatrix();

		p.rotateY(-mq4);
		p.translate(-32, -271, 0);
		p.pushMatrix();
		p.scale(0.12102564102564f);
		p.shape(ax4);
		p.popMatrix();

		p.rotateX(mq5);
		p.translate(32, -36.5f, 0);
		p.pushMatrix();
		p.scale(0.062105263157895f);
		p.shape(ax5);
		p.popMatrix();

		p.rotateY(-mq6);
		p.pushMatrix();
		p.scale(0.032f);
		p.shape(ax6);
		p.popMatrix();
		p.translate(0, -15f, 0);
		p.rotateX(p.HALF_PI);
		//print.show(step);
	}

	void drawGrid() {

		p.fill(30);
		p.noStroke();
		//// world GRID
		if (enableGrid) {
			p.stroke(32, 80);
			for (int i = 0; i < 101; i++) {
				int spc = i * 200;
				p.pushMatrix();
				p.strokeWeight(1);
				p.rotateX(-p.HALF_PI);
				p.translate(-10000, 0, -10000);
				p.line(spc, 0, 0, spc, 0, 20000); // z
				p.line(0, 0, spc, 20000, 0, spc); // x
				p.popMatrix();
			}
			p.stroke(255);
		}
	}

	void loadParts() {
		base = p.loadShape("ax0.obj");
		ax1 = p.loadShape("ax1.obj");
		ax2 = p.loadShape("ax2.obj");
		ax3 = p.loadShape("ax3.obj");
		ax4 = p.loadShape("ax4.obj");
		ax5 = p.loadShape("ax5.obj");
		ax6 = p.loadShape("ax6.obj");
		extr = p.loadShape("extruder.obj");
		ax1.disableStyle();
		ax2.disableStyle();
		ax3.disableStyle();
		ax4.disableStyle();
		ax5.disableStyle();
		ax6.disableStyle();
		extr.disableStyle();
	}

	void setCam() {
		// p.pointLight(190, 190, 190, -8000, 9000, 5000);
		scene.camera().setPosition(800, -1050, 700);
		scene.camera().setOrientation(0, -1.6f);
		scene.camera().lookAt(100, 0, 250);
	}

	void setLights() {
		p.ambientLight(40, 40, 40);
		p.pointLight(220, 220, 220, -1000, 1000, -2000);
		p.directionalLight(255, 255, 255, -1, 1, -1);
	}

	void gui() {
		// Disable depth test to draw 2d on top
		p.hint(p.DISABLE_DEPTH_TEST);
		p.perspective();
		p.camera();
		p.noLights();
		cp5.draw();

		// Re-enble depth test
		p.hint(p.ENABLE_DEPTH_TEST);
	}

	void setupScene() {
		scene.setRadius(5000);
		scene.setVisualHints(Scene.PICKING);
		scene.drawAxes(800);
		scene.flip();

		scene.eyeFrame().setDamping(0.9f);
		scene.eyeFrame().setWheelSensitivity(10); // zoom empfindlichkeit
	}

	public void loadTextLabel() {
		PFont pf = p.createFont("Verdana", 13);
		ControlFont font = new ControlFont(pf);
		cp5.setFont(font);

		cp5.addButton("Reset Cam").setPosition(20, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);
		
		cp5.addButton("Move Home").setPosition(1390, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);
		
		cp5.addButton("Move Rest").setPosition(1390, 80).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);

		cp5.addButton("saveFrame").setPosition(150, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);

		cp5.addButton("Create Traj").setPosition(280, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);

		cp5.addButton("Simulate Traj").setPosition(410, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);

		cp5.addButton("Run Traj").setPosition(1520, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);

		cp5.addButton("Show Points").setPosition(540, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);

		cp5.addButton("Exit").setPosition(1650, 20).setColorBackground(p.color(100, 100, 100))
				.setColorForeground(p.color(100, 100, 100)).setSize(120, 50);

	}
}
