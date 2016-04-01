import KinectPV2.KJoint;
import gab.opencv.*;
import KinectPV2.*;
import java.awt.*;
PShape s;
boolean flying = false;
OpenCV opencv;
KinectPV2 kinect;
KJoint[] joints;
float polygonFactor = 1;
int threshold = 10;
int maxD = 2500; 
int minD = 50;
boolean contourBodyIndex = false;
int counter = 1;
float noiseScale = 0.0;
int moreNoise = 1;
float Scale=300.0;
float yoff = 0.0;  
float xoff = 0; 
float p=1;
int mode=0;
boolean firstTime = true;

float diag;

void setup() {
  size(2000, 1000, P3D);
  //fullScreen(P3D);
  diag = sq(width)+sq(height);

  smooth();
  opencv = new OpenCV(this, 512, 424);
  kinect = new KinectPV2(this);
  kinect.enableDepthImg(true);
  kinect.enableBodyTrackImg(true);
  kinect.enablePointCloud(true);
  kinect.enableSkeleton3DMap(true);
  kinect.init();
}
int state=0;
void draw() {
  background(0);
  pushMatrix();
  translate(width/2, height/2, 0);
  scale(Scale, Scale, 1);
  rotateX(PI);
  rotateY(PI);
  popMatrix();
  contour();
}
void keyPressed() {
  if (key == 'a') {
    counter++;
  } else if (key == 's') {
    moreNoise++;
  } else if (key == 'd') {
    counter--;
  }
}

void contour() {
  PVector center = new PVector(0,0);
  int q=770;
  int w=270;
  pushMatrix();
  scale(2);
  translate(-280, 20);
  translate(q, w);
  strokeWeight(3);
  noFill();
  if (contourBodyIndex) {
    opencv.blur(80);
    opencv.gray();
    opencv.threshold(threshold);    
    PImage a=kinect.getBodyTrackImage();
    opencv.loadImage(a);
  } else {
    opencv.blur(80);
    opencv.gray();
    opencv.threshold(threshold);
    PImage b=kinect.getPointCloudDepthImage();
    opencv.loadImage(b);
  }

  ArrayList<Contour> contours = opencv.findContours(false, false);
  ArrayList<PVector> z=new ArrayList<PVector>();
  if (contours.size() > 0) {
    for (Contour contour : contours) {
      Rectangle boundingBox = contour.getBoundingBox();
      center = new PVector((boundingBox.width/2+boundingBox.x)*(width/512), (boundingBox.height/2+boundingBox.y)*(height/424));
      println(center.x, center.y);

      contour.setPolygonApproximationFactor(100);
      if (contour.numPoints() > 50) { 
        stroke(255, 100);
        strokeWeight(10);
        for (int i=0; i<contour.getPoints().size(); i++) {
          z.add(contour.getPoints().get(i));
        }
        for (int i=0; i<z.size(); i+=1) {
          pushMatrix();
          scale(p); 
          beginShape();
          point(1024-z.get(i).x-q, z.get(i).y-w);
          endShape();
     
          popMatrix();
        }
      }
    }
  }
    pushMatrix();
    if (contours.size() > 0) {
      for (Contour contour : contours) {
        contour.setPolygonApproximationFactor(polygonFactor);
        if (contour.numPoints() > 100) {
          int red = 0;
          int green = 200;
          int blue = 200;
          float offset = 1.0;
          for (int i=0; i<1; i++) {
            stroke(red, green, blue);
            s = createShape();
            s.beginShape();
            for (int j=0; j<contour.getPolygonApproximation().getPoints().size(); j++) {
              PVector point = contour.getPolygonApproximation().getPoints().get(j);
              //noiseScale += .01;
              //float n = noise(noiseScale) * moreNoise;
              //s.vertex((1024-point.x-q) +n, (point.y-w)+n);
            }
            firstTime = false;
            s.endShape();
            s.scale(offset);
            shape(s);
            red += 25;
            green -= 30;
            blue -= 15;
            offset += .05;
          }
        }
      }
    }
    popMatrix();  
    popMatrix();
    for (int x = 0; x < width; x+=20) {
       for (int y = 0; y < height; y+=20) {

         float d = dist(x, y, center.x, center.y);
         float f = map(d, 0, diag, 255, 0);
         fill(f);

          strokeWeight(2);
          stroke(0);
         rect(x, y, 20, 20);
       }
      }
    kinect.setLowThresholdPC(minD);
    kinect.setHighThresholdPC(maxD);
  }