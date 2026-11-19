// Processing 4 visualization driven by a native host build of the real Main.ino
// control loop; arena physics and sensor readings remain simulation models.

import java.io.*;

final int STOP=0, SEARCH=1, APPROACH=2, AVOID=3, CARRY=4, GOAL=5;
final int FLOOR_UNKNOWN=0, FLOOR_BLACK=1, FLOOR_RED=2, FLOOR_BLUE=3;
final float FIELD_W=240, FIELD_H=150, BORDER=5;
final float HILL_X=120, HILL_Y=81, HILL_RX=35.5, HILL_RY=35, HILL_H=15;
final int TARGET_RANGE=100, OBSTACLE_LIMIT=15;
final float FORWARD_SPEED=100, BACKWARD_SPEED=80, ROTATE_SPEED=150;
final float ROTATE_180_TIME_MS=2000, SWEEP_MS=ROTATE_180_TIME_MS*150.0/180.0;
// Map compass: from the blue goal line toward the field center is North.
final float MAP_NORTH_HEADING=180;
final String[] COMPASS_LABELS={"N","E","S","W"};
final int[] COMPASS_LABEL_X={-21,-7,7,-7}, COMPASS_LABEL_Y={0,-8,0,8};
final float[] compassScreenX=new float[4], compassScreenY=new float[4];
final float SOUTH_HEADING=180, HEADING_TOLERANCE=12;
final float CARRIED_CUP_OFFSET=9;
final float FIXED_DT=1.0/60.0;

Robot bot;
ArrayList<Cup> cups=new ArrayList<Cup>();
boolean paused=false, showSensor=true;
float simSpeed=1, accumulator=0, previousMs=0;
// Fixed opposite-side camera: north-side view, looking toward map south.
final float CAMERA_YAW=0, CAMERA_PITCH=0.78, CAMERA_DISTANCE=350;
float panX=0,panY=0;
float mouseDownX=0,mouseDownY=0;
int stage=0; // button: color setup, compass calibration, run toggle
int teamColor=FLOOR_BLUE;
int scoreBlue=0, scoreRed=0;
String lastEvent="Power on: STOP";
volatile File controllerExecutable;
volatile String controllerBuildError="";

class Cup {
  float x,y; boolean delivered=false, mountain=false;
  Cup(float x_,float y_,boolean hill_) { x=x_; y=y_; mountain=hill_; }
  float z() { return terrainHeight(x,y)+3.6; }
}

class Robot {
  // Contest figure 2: blue Robot 1 starts at the lower-right mark, facing inward.
  float x=212,y=136,heading=180,left=0,right=0;
  int state=STOP, searchStep=0;
  boolean enabled=false;
  boolean compassReady=false, compassCalibrated=false;
  boolean headingReferenceReady=false;
  float headingDelta=-1;
  boolean targetDetected=false, obstacle=false;
  int distance=-1,targetDistance=-1,floor=FLOOR_UNKNOWN;
  float matchTime=0;
  float previousState=STOP;
  boolean fallbackController=false;
  float headingReference=-1,searchPhaseStart=0,goalReturnHeading=-1,goalSequenceStart=0;
  int searchHeadingIndex=0,goalSequencePhase=0,ownColorSamples=0;
  boolean searchForwarding=true,carryTurnFinished=false;
  Cup sensedCup=null, carriedCup=null;
  Process controllerProcess;
  BufferedWriter controllerInput;
  BufferedReader controllerOutput;
  String pendingCommand="-";

  void setState(int s) {
    if(state==s) return;
    if(s==CARRY && carriedCup==null) carriedCup=sensedCup;
    lastEvent=name(state)+" → "+name(s);
    state=s;
  }
  void setDrive(float l,float r) { left=constrain(l,-250,250); right=constrain(r,-250,250); }
  void stop() { setDrive(0,0); }
  void forward(float s) { setDrive(s,s); }
  void backward(float s) { setDrive(-s,-s); }
  void rotate(float s) { setDrive(s,-s); }

  void startController() {
    if(controllerExecutable==null || controllerBuildError.length()>0 ||
       (controllerProcess!=null && controllerProcess.isAlive())) return;
    try {
      controllerProcess=new ProcessBuilder(controllerExecutable.getAbsolutePath()).start();
      controllerInput=new BufferedWriter(new OutputStreamWriter(controllerProcess.getOutputStream()));
      controllerOutput=new BufferedReader(new InputStreamReader(controllerProcess.getInputStream()));
    } catch(Exception error) {
      controllerBuildError="Could not start .ino controller: "+error.getMessage();
    }
  }

  void stopController() {
    try { if(controllerInput!=null) controllerInput.close(); } catch(Exception ignored) {}
    if(controllerProcess!=null) controllerProcess.destroy();
  }

  void buttonPress() {
    if(stage>=2 && !enabled) heading=MAP_NORTH_HEADING;
    stop();
    pendingCommand="BUTTON";
    lastEvent="BUTTON: sent to Main.ino";
  }

  void command(String cmd) {
    if(cmd.equals("START") && stage>=2 && !enabled) heading=MAP_NORTH_HEADING;
    pendingCommand=cmd;
    lastEvent=cmd+": sent to Main.ino";
  }

  int raycastDistance() {
    Cup best=null; float alongBest=101;
    float dx=cos(radians(heading)),dy=sin(radians(heading));
    for(Cup c:cups) if(!c.delivered && !c.mountain) {
      float vx=c.x-x,vy=c.y-y,along=vx*dx+vy*dy,side=abs(vx*dy-vy*dx);
      if(along>0 && along<=100 && side<5.6 && along<alongBest) { best=c; alongBest=along; }
    }
    sensedCup=best;
    if(best!=null) {
      // 接近中は車体前端がカップに届いた時点で接触扱いにする。
      if(best==carriedCup || (state==APPROACH && alongBest<CARRIED_CUP_OFFSET)) return 0;
      int cupDistance=round(dist(x,y,best.x,best.y));
      return cupDistance<=2?0:cupDistance;
    }
    // シミュレーションの超音波反射対象はカップのみ。フィールド境界はエコーとして返さない。
    return -1;
  }

  void tick(float dt) {
    matchTime+=dt;
    distance=raycastDistance();
    floor=surfaceAt(x,y);
    obstacle=distance>0 && distance<OBSTACLE_LIMIT;
    if(fallbackController) { tickFallback(); return; }
    if(controllerProcess==null || !controllerProcess.isAlive()) { stop(); return; }
    try {
      controllerInput.write(round(matchTime*1000)+" "+distance+" "+floor+" "+heading+" "+pendingCommand);
      controllerInput.newLine(); controllerInput.flush(); pendingCommand="-";
      String output=controllerOutput.readLine();
      if(output==null) { controllerBuildError="The .ino controller stopped unexpectedly."; stop(); return; }
      String[] values=split(output,',');
      if(values.length>=6) {
        int oldState=state;
        setState(parseInt(values[0]));
        left=parseFloat(values[1]); right=parseFloat(values[2]);
        enabled=parseInt(values[3])!=0;
        targetDetected=parseInt(values[4])!=0;
        int measuredDistance=parseInt(values[5]);
        targetDistance=targetDetected ? (measuredDistance>=0?measuredDistance:distance) : -1;
        if(values.length>=10) {
          int previousStage=stage;
          stage=parseInt(values[6]);
          headingDelta=parseFloat(values[7]);
          heading=wrap360(parseFloat(values[8]));
          compassCalibrated=stage>=2;
          headingReferenceReady=headingDelta>=0;
          if(!values[9].equals("-")) lastEvent=values[9];
          else if(previousStage!=stage) lastEvent="Stage changed: "+stage;
        }
        if(oldState==GOAL && state==SEARCH && carriedCup!=null) {
          carriedCup.delivered=true;
          scoreCup(carriedCup,teamColor);
          carriedCup=null;
        }
      }
    } catch(Exception error) {
      controllerBuildError="Controller communication failed: "+error.getMessage();
      stop();
    }
    if(state==CARRY && previousState!=CARRY) lastEvent="CARRY: simulated 440Hz entry beep";
    previousState=state;
  }

  void tickFallback() {
    String command=pendingCommand;
    pendingCommand="-";
    if(command.equals("BUTTON")) {
      if(stage==0) { stage=1; lastEvent="COLOR_READY"; }
      else if(stage==1) {
        float angular=(-30.0/8.5)*6.25;
        heading=wrap360(heading+degrees(angular));
        compassCalibrated=true; stage=2; lastEvent="COMPASS_CALIBRATED";
      } else if(!enabled) {
        captureFallbackReference(); lastEvent="STARTED";
      } else { enabled=false; state=STOP; stop(); lastEvent="STOPPED"; }
    } else if(command.equals("START")) {
      if(stage>=2) { captureFallbackReference(); lastEvent="STARTED"; }
      else lastEvent="CALIBRATE_FIRST";
    } else if(command.equals("STOP")) {
      enabled=false; state=STOP; stop(); lastEvent="STOPPED";
    }

    headingDelta=headingReference<0?-1:wrap360(heading-headingReference);
    if(!enabled) { state=STOP; stop(); return; }

    distance=raycastDistance();
    floor=surfaceAt(x,y);
    obstacle=distance>0 && distance<OBSTACLE_LIMIT;
    if(state==SEARCH) {
      if(distance>=0 && distance<=TARGET_RANGE) {
        targetDetected=true; state=APPROACH; stop(); return;
      }
      if(searchForwarding) {
        if(matchTime-searchPhaseStart<1.0) { forward(FORWARD_SPEED); return; }
        stop(); searchForwarding=false; searchHeadingIndex=0; return;
      }
      float[] headings={45,90,135,180,225,270,315,0};
      if(fallbackTurnTo(headings[searchHeadingIndex],HEADING_TOLERANCE)) {
        searchHeadingIndex++;
        if(searchHeadingIndex>=headings.length) {
          searchHeadingIndex=0; searchForwarding=true; searchPhaseStart=matchTime;
        }
      }
      return;
    }
    if(state==APPROACH) {
      if(distance<0) { targetDetected=false; state=SEARCH; stop(); return; }
      if(distance<=3) {
        targetDetected=false; setState(CARRY);
        carryTurnFinished=false; ownColorSamples=0; stop(); return;
      }
      forward(100); return;
    }
    if(state==CARRY) {
      if(floor==teamColor) ownColorSamples++; else ownColorSamples=0;
      if(ownColorSamples>=3) {
        goalReturnHeading=headingDelta; goalSequencePhase=0; state=GOAL; stop(); return;
      }
      if(!carryTurnFinished) {
        if(fallbackTurnTo(SOUTH_HEADING,HEADING_TOLERANCE)) {
          carryTurnFinished=true; stop();
        }
        return;
      }
      forward(FORWARD_SPEED); return;
    }
    if(state==GOAL) {
      if(goalSequencePhase==0) {
        goalSequenceStart=matchTime; goalSequencePhase=1; backward(BACKWARD_SPEED); return;
      }
      if(goalSequencePhase==1 && matchTime-goalSequenceStart<2.0) {
        backward(BACKWARD_SPEED); return;
      }
      goalSequencePhase=2;
      if(goalReturnHeading<0) { stop(); return; }
      if(!fallbackTurnTo(wrap360(goalReturnHeading+180),HEADING_TOLERANCE)) return;
      if(carriedCup!=null) {
        carriedCup.delivered=true; scoreCup(carriedCup,teamColor); carriedCup=null;
      }
      state=SEARCH; goalSequencePhase=0; carryTurnFinished=false;
      searchForwarding=true; searchPhaseStart=matchTime; searchHeadingIndex=0;
      lastEvent="GOAL: backed up and resumed SEARCH"; stop(); return;
    }
    stop();
  }

  void captureFallbackReference() {
    headingReference=heading; headingDelta=0; headingReferenceReady=true;
    enabled=true; state=SEARCH; searchForwarding=true;
    searchHeadingIndex=0; searchPhaseStart=matchTime;
  }

  boolean fallbackTurnTo(float target,float tolerance) {
    float error=target-headingDelta;
    while(error>180) error-=360;
    while(error< -180) error+=360;
    if(abs(error)<=tolerance) { stop(); return true; }
    setDrive(error<0?ROTATE_SPEED:-ROTATE_SPEED,
             error<0?-ROTATE_SPEED:ROTATE_SPEED);
    return false;
  }

  void physics(float dt) {
    float vl=left*.075,vr=right*.075,wheelBase=8.5;
    float linear=(vl+vr)*.5,angular=(vr-vl)/wheelBase;
    heading=wrap360(heading+degrees(angular*dt));
    float nx=x+cos(radians(heading))*linear*dt,ny=y+sin(radians(heading))*linear*dt;
    Cup c=nearestCupAhead();
    float cx,cy;
    if(c!=null && linear>0) {
      if(carriedCup!=null) {
        cx=(c.x-x)*sin(radians(heading))-(c.y-y)*cos(radians(heading));
        cy=cx<0?90:-90;
        c.x=constrain(c.x+cos(radians(heading+cy))*linear*dt,-4,FIELD_W+4);
        c.y=constrain(c.y+sin(radians(heading+cy))*linear*dt,BORDER+2,FIELD_H-BORDER-2);
      } else {
        cx=c.x+cos(radians(heading))*linear*dt;
        cy=c.y+sin(radians(heading))*linear*dt;
        if(cx< -3) { c.delivered=true; scoreCup(c,FLOOR_RED); }
        else if(cx>FIELD_W+3) { c.delivered=true; scoreCup(c,FLOOR_BLUE); }
        else { c.x=constrain(cx,-4,FIELD_W+4); c.y=constrain(cy,BORDER+2,FIELD_H-BORDER-2); }
      }
    }
    float oldZ=terrainHeight(x,y),newZ=terrainHeight(nx,ny);
    float slopeFactor=1.0/(1.0+abs(newZ-oldZ)*0.10);
    x=constrain(x+(nx-x)*slopeFactor,-10,FIELD_W+10);
    y=constrain(y+(ny-y)*slopeFactor,-10,FIELD_H+10);
    if(carriedCup!=null && state!=GOAL) {
      carriedCup.x=x+cos(radians(heading))*CARRIED_CUP_OFFSET;
      carriedCup.y=y+sin(radians(heading))*CARRIED_CUP_OFFSET;
    }
  }
  Cup nearestCupAhead() {
    Cup best=null; float bd=9;
    float dx=cos(radians(heading)),dy=sin(radians(heading));
    for(Cup c:cups) if(!c.delivered && c!=carriedCup && (state!=APPROACH || c!=sensedCup)) {
      float vx=c.x-x,vy=c.y-y,along=vx*dx+vy*dy,side=abs(vx*dy-vy*dx);
      if(along>0 && along<bd && side<5.8) { best=c; bd=along; }
    }
    return best;
  }
}

void setup() {
  size(1440,900,P3D); smooth(8); frameRate(60);
  textFont(createFont("Yu Gothic",15,true));
  controllerBuildError="Building native .ino controller...";
  resetSimulation(); previousMs=millis();
  thread("buildController");
}

void buildController() {
  try {
    File root=new File(sketchPath()).getParentFile();
    File host=new File(root,"System18Simulator/host");
    File main=new File(root,"Main");
    controllerExecutable=new File(System.getProperty("java.io.tmpdir"),"system18-controller-"+System.nanoTime()+".exe");
    ArrayList<String> command=new ArrayList<String>();
    command.add("g++"); command.add("-std=c++17"); command.add("-x"); command.add("c++");
    command.add("-I"); command.add(host.getAbsolutePath());
    command.add("-I"); command.add(main.getAbsolutePath());
    command.add(new File(host,"HostSimulator.cpp").getAbsolutePath());
    command.add(new File(host,"HostAdapters.cpp").getAbsolutePath());
    command.add("-o"); command.add(controllerExecutable.getAbsolutePath());
    Process compiler=new ProcessBuilder(command).directory(root).redirectErrorStream(true).start();
    BufferedReader log=new BufferedReader(new InputStreamReader(compiler.getInputStream()));
    StringBuilder output=new StringBuilder(); String line;
    while((line=log.readLine())!=null) output.append(line).append('\n');
    int result=compiler.waitFor();
    if(result!=0) {
      controllerBuildError="Controller build failed: "+output.toString();
      return;
    }
    controllerBuildError="";
  } catch(Exception error) {
    controllerBuildError="Controller build failed: "+error.getMessage();
  }
}

void resetSimulation() {
  if(bot!=null) bot.stopController();
  cups.clear(); scoreBlue=scoreRed=0;
  // Match the 11 cup markers in contest overview Figure 2: ten on the flat
  // field and one on the hill summit. Keep this illustrated layout repeatable.
  cups.add(new Cup(51,31,false)); cups.add(new Cup(90,41,false));
  cups.add(new Cup(150,41,false)); cups.add(new Cup(189,31,false));
  cups.add(new Cup(51,75,false)); cups.add(new Cup(189,75,false));
  cups.add(new Cup(51,119,false)); cups.add(new Cup(90,109,false));
  cups.add(new Cup(150,109,false)); cups.add(new Cup(189,119,false));
  cups.add(new Cup(HILL_X,HILL_Y,true));
  bot=new Robot(); bot.startController(); paused=false; accumulator=0; stage=0; lastEvent="Power on: STOP";
}

void draw() {
  float now=millis(),dt=min(.1,(now-previousMs)/1000.0); previousMs=now;
  if(bot.controllerProcess==null && controllerExecutable!=null && controllerBuildError.length()==0) bot.startController();
  if(bot.controllerProcess==null && !controllerBuildError.startsWith("Building native")) bot.fallbackController=true;
  if(!paused) {
    accumulator+=dt*simSpeed;
    while(accumulator>=FIXED_DT) { bot.tick(FIXED_DT); bot.physics(FIXED_DT); accumulator-=FIXED_DT; }
  }
  background(218,228,235);
  drawScene(); drawOverlay();
}

void drawScene() {
  float cx=FIELD_W/2+panX,cy=FIELD_H/2+panY,cz=0;
  float horizontal=CAMERA_DISTANCE*cos(CAMERA_PITCH);
  float eyeX=cx+horizontal*sin(CAMERA_YAW);
  float eyeY=cy-horizontal*cos(CAMERA_YAW);
  float eyeZ=cz+CAMERA_DISTANCE*sin(CAMERA_PITCH);
  float viewY=cos(CAMERA_PITCH)*cos(CAMERA_YAW),viewZ=-sin(CAMERA_PITCH);
  float upDot=-viewY;
  float upY=-1-upDot*viewY,upZ=-upDot*viewZ;
  float upLength=sqrt(upY*upY+upZ*upZ);
  upY/=upLength; upZ/=upLength;
  camera(eyeX,eyeY,eyeZ,cx,cy,cz,0,upY,upZ);
  perspective(PI/3.0,width/(float)height,1,3000);
  ambientLight(150,150,150); directionalLight(210,210,210,-0.4,0.3,-1);
  pushMatrix();
  translate(FIELD_W/2,FIELD_H/2,0);
  // Keep diagram coordinates upright: y=0 is the far/top side, y grows down.
  scale(2.3,2.3,2.3);
  translate(-FIELD_W/2,-FIELD_H/2,0);
  drawArena(); drawCompassMarker(); drawMountain();
  for(Cup c:cups) if(!c.delivered) drawCup(c);
  drawRobot();
  if(showSensor && bot.distance>0) {
    stroke(20,165,110); strokeWeight(.45);
    float len=bot.distance;
    line(bot.x,bot.y,terrainHeight(bot.x,bot.y)+4,
         bot.x+cos(radians(bot.heading))*len,bot.y+sin(radians(bot.heading))*len,terrainHeight(bot.x,bot.y)+4);
  }
  popMatrix();
}

void drawArena() {
  noStroke(); fill(249); beginShape(QUADS);
  vertex(0,0,0); vertex(FIELD_W,0,0); vertex(FIELD_W,FIELD_H,0); vertex(0,FIELD_H,0); endShape();
  fill(27); beginShape(QUADS);
  vertex(0,0,.15);vertex(FIELD_W,0,.15);vertex(FIELD_W,BORDER,.15);vertex(0,BORDER,.15);
  vertex(0,FIELD_H-BORDER,.15);vertex(FIELD_W,FIELD_H-BORDER,.15);vertex(FIELD_W,FIELD_H,.15);vertex(0,FIELD_H,.15);
  vertex(0,0,.15);vertex(BORDER,0,.15);vertex(BORDER,FIELD_H,.15);vertex(0,FIELD_H,.15);
  vertex(FIELD_W-BORDER,0,.15);vertex(FIELD_W,0,.15);vertex(FIELD_W,FIELD_H,.15);vertex(FIELD_W-BORDER,FIELD_H,.15);
  endShape();
  fill(218,53,53); beginShape(QUADS); vertex(0,BORDER,.3);vertex(BORDER,BORDER,.3);vertex(BORDER,FIELD_H-BORDER,.3);vertex(0,FIELD_H-BORDER,.3);endShape();
  fill(45,95,225); beginShape(QUADS); vertex(FIELD_W-BORDER,BORDER,.3);vertex(FIELD_W,BORDER,.3);vertex(FIELD_W,FIELD_H-BORDER,.3);vertex(FIELD_W-BORDER,FIELD_H-BORDER,.3);endShape();
  stroke(155); strokeWeight(.2); noFill(); rect(0,0,FIELD_W,FIELD_H);
}

// 地図上のNorthは青ライン中央からフィールド中心へ向かう方向。
void drawCompassMarker() {
  float x=FIELD_W-BORDER-4, y=FIELD_H/2, z=.9;
  pushStyle();
  stroke(22,42,58); strokeWeight(.9);
  line(x,y,z,x-16,y,z);
  line(x-16,y,z,x-11,y-3,z);
  line(x-16,y,z,x-11,y+3,z);
  popStyle();
  // 地図上の位置を画面へ投影し、文字は画面に正対させる。
  for(int i=0;i<COMPASS_LABELS.length;i++) {
    compassScreenX[i]=screenX(x+COMPASS_LABEL_X[i],y+COMPASS_LABEL_Y[i],z);
    compassScreenY[i]=screenY(x+COMPASS_LABEL_X[i],y+COMPASS_LABEL_Y[i],z);
  }
}

void drawMountain() {
  int rings=32,segments=72;
  noStroke();
  for(int r=0;r<rings;r++) {
    float q0=r/(float)rings,q1=(r+1)/(float)rings;
    beginShape(QUAD_STRIP);
    for(int i=0;i<=segments;i++) {
      float a=TWO_PI*i/segments;
      float x0=HILL_X+HILL_RX*q0*cos(a),y0=HILL_Y+HILL_RY*q0*sin(a);
      float x1=HILL_X+HILL_RX*q1*cos(a),y1=HILL_Y+HILL_RY*q1*sin(a);
      fill(150+q0*35,158+q0*30,150+q0*25); vertex(x0,y0,terrainHeight(x0,y0));
      fill(150+q1*35,158+q1*30,150+q1*25); vertex(x1,y1,terrainHeight(x1,y1));
    }
    endShape();
  }
  // Thin contour rings make both slope bands visible from the default camera.
  stroke(125,135,128,150); strokeWeight(.25); noFill();
  for(float q=.12;q<=1.001;q+=.12) {
    beginShape(); for(int i=0;i<=100;i++) { float a=TWO_PI*i/100; float x=HILL_X+HILL_RX*q*cos(a),y=HILL_Y+HILL_RY*q*sin(a); vertex(x,y,terrainHeight(x,y)+.12); } endShape();
  }
}

void drawCup(Cup c) {
  pushMatrix(); translate(c.x,c.y,c.z()); noStroke();
  if(c.mountain) fill(255,150,20); else fill(237,194,35);
  sphereDetail(14); sphere(3.6); popMatrix();
}

void drawRobot() {
  float h=radians(bot.heading),dx=cos(h),dy=sin(h);
  float front=terrainHeight(bot.x+dx*5,bot.y+dy*5),rear=terrainHeight(bot.x-dx*5,bot.y-dy*5);
  float leftZ=terrainHeight(bot.x-dy*4.5,bot.y+dx*4.5),rightZ=terrainHeight(bot.x+dy*4.5,bot.y-dx*4.5);
  float support=max(terrainHeight(bot.x,bot.y),max((front+rear)*.5,(leftZ+rightZ)*.5));
  pushMatrix(); translate(bot.x,bot.y,support+2.8); rotateZ(h);
  rotateY(atan2(front-rear,10)); rotateX(-atan2(leftZ-rightZ,9));
  noStroke(); fill(48,115,160); box(10,9,4);
  fill(28); pushMatrix(); translate(0,-5.4,-.5); box(7,2,4.5); popMatrix();
  pushMatrix(); translate(0,5.4,-.5); box(7,2,4.5); popMatrix();
  fill(240,70,55); beginShape(TRIANGLES); vertex(7,0,1);vertex(4,-2.5,1);vertex(4,2.5,1);endShape();
  popMatrix();
}

void drawOverlay() {
  // 標準カメラでは z=0 の座標が左上原点の画面座標に一致する。
  hint(DISABLE_DEPTH_TEST); camera(); perspective(); noLights();
  pushStyle();
  fill(22,42,58); textAlign(CENTER,CENTER); textSize(15);
  for(int i=0;i<COMPASS_LABELS.length;i++) {
    text(COMPASS_LABELS[i],compassScreenX[i],compassScreenY[i]);
  }
  popStyle();
  drawHud(); drawStateBanner();
  hint(ENABLE_DEPTH_TEST);
}

void drawHud() {
  float x=18;
  fill(18,30,42); noStroke(); rect(x,18,365,735,12);
  fill(255); textSize(22); text("System18 Simulator",x+22,60);
  textSize(15); fill(210,224,232);
  text("STATE: "+name(bot.state),x+22,98);
  text("Transition: "+lastEvent,x+22,126);
  text("Stage: "+(stage==0?"color setup":stage==1?"compass calibration": "ready / running"),x+22,158);
  text("Enabled: "+bot.enabled+"    Compass calibrated: "+bot.compassCalibrated,x+22,184);
  text("Floor: "+floorName(bot.floor)+"    Obstacle flag: "+bot.obstacle,x+22,218);
  text("Distance: "+bot.distance+" cm    Target: "+bot.targetDetected,x+22,246);
  text("Heading: "+nf(bot.heading,1,1)+"°    Motor: "+round(bot.left)+" / "+round(bot.right),x+22,274);
  text("Facing: "+facingName(bot.headingDelta)+"    Delta: "+(bot.headingDelta>=0?nf(bot.headingDelta,1,1)+"°":"NA"),x+22,302);
  String controllerStatus=bot.fallbackController?"fallback":"Main.ino";
  text("Controller: "+controllerStatus+" / "+(bot.fallbackController?"active":bot.controllerProcess!=null && bot.controllerProcess.isAlive()?"running":"offline"),x+22,330);
  text("Match time: "+nf(bot.matchTime,1,1)+" s",x+22,358);
  text("Score RED "+scoreRed+"     BLUE "+scoreBlue,x+22,386);
  fill(245,218,120); text("Controls",x+22,427); fill(225);
  text("B / click: color, compass, start (3 presses)",x+22,455);
  text("S: START command     X: STOP command",x+22,481);
  text("Space: pause     R: reset layout",x+22,507);
  text("+ / -: speed x"+nf(simSpeed,1,2)+"     V: sensor ray",x+22,533);
  text("1: black test   2: red test   3: blue test",x+22,559);
  text("Right-drag: move map; view fixed opposite",x+22,585);
  text("Map North: blue line toward field center",x+22,619);
  text("Blue start: lower-right; 11 cups total",x+22,645);
  text("Hill: 15 cm high, 71 x 70 cm elliptical base",x+22,671);
  text(bot.fallbackController?"Approximate fallback control; 3D physics.":"Main.ino control; approximate 3D physics.",x+22,697);
  if(controllerBuildError.length()>0) { fill(255,150,130); textSize(11); text(controllerBuildError,x+22,715,320,34); }
}

void drawStateBanner() {
  String label="STATE: "+name(bot.state);
  pushStyle(); textSize(22);
  float boxWidth=max(190,textWidth(label)+32);
  noStroke(); fill(18,30,42); rect(width-boxWidth-20,18,boxWidth,48,8);
  fill(255); textAlign(CENTER,CENTER);
  text(label,width-boxWidth/2-20,42);
  popStyle();
}

void keyPressed() {
  if(key==' ') paused=!paused;
  else if(key=='r'||key=='R') resetSimulation();
  else if(key=='b'||key=='B') bot.buttonPress();
  else if(key=='s'||key=='S') bot.command("START");
  else if(key=='x'||key=='X') bot.command("STOP");
  else if(key=='v'||key=='V') showSensor=!showSensor;
  else if(key=='+'||key=='=') simSpeed=min(8,simSpeed*2);
  else if(key=='-'||key=='_') simSpeed=max(.25,simSpeed*.5);
  else if(key=='1') { bot.x=120; bot.y=3; lastEvent="Test placement: black boundary"; }
  else if(key=='2') { bot.x=3; bot.y=75; lastEvent="Test placement: opponent RED line"; }
  else if(key=='3') { bot.x=FIELD_W-3; bot.y=75; lastEvent="Test placement: own BLUE line"; }
}

void mousePressed() { mouseDownX=mouseX; mouseDownY=mouseY; }
void mouseDragged() {
  if(mouseButton==RIGHT) {
    panX-= (mouseX-pmouseX)*.12;
    panY-= (mouseY-pmouseY)*.12;
  }
}
void mouseReleased() {
  if(mouseButton==LEFT && dist(mouseDownX,mouseDownY,mouseX,mouseY)<5 && mouseX<width-390) bot.buttonPress();
}
int surfaceAt(float x,float y) {
  if(x>=0&&x<=BORDER&&y>BORDER&&y<FIELD_H-BORDER) return FLOOR_RED;
  if(x>=FIELD_W-BORDER&&x<=FIELD_W&&y>BORDER&&y<FIELD_H-BORDER) return FLOOR_BLUE;
  if(x<BORDER||x>FIELD_W-BORDER||y<BORDER||y>FIELD_H-BORDER) return FLOOR_BLACK;
  return FLOOR_UNKNOWN;
}
float terrainHeight(float x,float y) {
  float q=sqrt(sq((x-HILL_X)/HILL_RX)+sq((y-HILL_Y)/HILL_RY));
  if(q>=1) return 0;
  // Smooth two-band mound: broad outer grade, steeper inner grade, small summit.
  if(q<=.08) return HILL_H;
  if(q<=.45) {
    float t=(q-.08)/(.45-.08); t=t*t*(3-2*t);
    return lerp(HILL_H,6,t);
  }
  float t=(q-.45)/(1-.45); t=t*t*(3-2*t);
  return lerp(6,0,t);
}
void scoreCup(Cup c,int goal) {
  int points=c.mountain?3:1;
  if(goal==FLOOR_BLUE) scoreBlue+=points; else scoreRed+=points;
}
boolean isOpponentColor(int c) { return (teamColor==FLOOR_BLUE&&c==FLOOR_RED)||(teamColor==FLOOR_RED&&c==FLOOR_BLUE); }
String floorName(int f) { if(f==FLOOR_BLACK)return "BLACK"; if(f==FLOOR_RED)return "RED"; if(f==FLOOR_BLUE)return "BLUE"; return "UNKNOWN"; }
String name(int s) { if(s==STOP)return "STOP"; if(s==SEARCH)return "SEARCH"; if(s==APPROACH)return "APPROACH"; if(s==AVOID)return "AVOID"; if(s==CARRY)return "CARRY"; if(s==GOAL)return "GOAL"; return "?"; }
float wrap360(float a) { while(a<0)a+=360; while(a>=360)a-=360; return a; }
String facingName(float heading) {
  if(heading<0) return "Unknown";
  if(heading>=315||heading<45) return "North";
  if(heading<135) return "East";
  if(heading<225) return "South";
  return "West";
}
