import java.util.ArrayList;

import processing.core.PApplet;
import processing.data.Table;
import processing.data.TableRow;

public class PrintPath {
	PApplet p;
	double x;
	double y;
	double z;

	ArrayList<Vert> list = new ArrayList<Vert>();
	Table listTraj;
	public PrintPath(PApplet _p, Table _list, double _x, double _y, double _z) {
		p = _p;
		listTraj = _list;
		x = _x;
		y = _y;
		z = _z;
		for(int i = 0; i < listTraj.getRowCount(); i++) {
			TableRow row = listTraj.getRow(i);
			list.add(new Vert(row.getDouble(0), row.getDouble(1), row.getDouble(2)));
		}
		
	}

	public void addVert(Jama.Matrix T) {
		list.add(new Vert(T.get(0, 3), T.get(1, 3), T.get(2, 3)));
	}
	
	public void show(int step) {
		
		p.stroke(0,0,200);
		TableRow row = listTraj.getRow(step);
		float x = row.getFloat(0);
		float y = row.getFloat(1);
		float z = row.getFloat(2);
		float xx = row.getFloat(3);
		float xy = row.getFloat(4);
		float xz = row.getFloat(5);
		float yx = row.getFloat(6);
		float yy = row.getFloat(7);
		float yz = row.getFloat(8);
		float zx = row.getFloat(9);
		float zy = row.getFloat(10);
		float zz = row.getFloat(11);
		
	    p.pushMatrix();
	    p.strokeWeight(1);
	    float size = 50;
	    p.stroke(255, 0, 0);
	    p.line(x, y, z, x+xx*size, y+xy*size, z+ xz*size);   
	    p.stroke(0, 255, 0); 
	    p.line(x, y, z, x+yx*size, y+yy*size, z+yz*size);
	    p.stroke(0, 0, 255);
	    p.line(x, y, z, x+zx*size, y+zy*size, z+zz*size);

	    p.popMatrix();
	    p.strokeWeight(3);
		for(int i = 0; i<step; i++) {
			p.point((float)list.get(i).x,(float)list.get(i).y,(float)list.get(i).z);
		}
	}

	public class Vert {
		double x;
		double y;
		double z;

		public Vert(double _x, double _y, double _z) {
			x = _x;
			y = _y;
			z = _z;
		}
	}
}

