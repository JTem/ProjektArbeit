import java.util.ArrayList;

import controlP5.*;
import processing.core.*;
import processing.data.Table;
import processing.data.TableRow;
import remixlab.proscene.*;

public class Main {

	PApplet p;
	PShape base, ax1, ax2, ax3, ax4, ax5, ax6;
	Scene scene;
	ControlP5 cp5;
	Jama.Matrix test_Trans;
	Trajectory trajectory;
	long last_time, current_time;
	Button ex;
	double time = 0;
	boolean enableGrid = true;
	boolean show_points = true;

	Table path;
	int step = 1;

	public Main(PApplet _p) {
		p = _p;
		cp5 = new ControlP5(p);
		scene = new Scene(p);

		setCam();
		setupScene();

		loadTextLabel();
		cp5.setAutoDraw(false);

	}

	public void display() {

		p.background(32);
		scene.drawAxes(215);
		p.strokeWeight(10);
		p.stroke(255);
		setLights();

		//runTraj2();
//		trajectory.show(step);
		drawGrid();
		//gui();

	}
	

	void drawGrid() {

		p.fill(190);
		p.noStroke();
		//// world GRID
		if (enableGrid) {
			p.stroke(255, 80);
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

	public void runTraj2() {

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

		cp5.addButton("Test Path").setPosition(150, 20).setColorBackground(p.color(100, 100, 100))
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
