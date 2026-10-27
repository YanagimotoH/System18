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
final float SOUTH_HEADING=180, HEADING_TOLERANCE=12;
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
  boolean targetDetected=false, obstacle=false;
  int distance=-1,targetDistance=-1,floor=FLOOR_UNKNOWN;
  float matchTime=0;
  float previousState=STOP;
  Cup carriedCup=null;
  Process controllerProcess;
  BufferedWriter controllerInput;
  BufferedReader controllerOutput;
  String pendingCommand="-";

  void setState(int s) { if(state!=s) lastEvent=name(state)+" → "+name(s); state=s; }
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
    if(stage==0) { stop(); stage=1; compassReady=true; lastEvent="COLOR_READY: simulated 2-second calibration"; return; }
    if(stage==1) { stop(); stage=2; compassCalibrated=true; lastEvent="COMPASS_CALIBRATED: simulated rotation calibration"; return; }
    if(!enabled) {
      headingReferenceReady=true;
      enabled=true; pendingCommand="START"; lastEvent="STARTED: heading reference captured";
    } else { enabled=false; pendingCommand="STOP"; stop(); lastEvent="STOPPED by button"; }
  }

  void command(String cmd) {
    if(cmd.equals("START")) {
      if(stage>=2 && compassReady) {
        headingReferenceReady=true; enabled=true; pendingCommand="START"; lastEvent="STARTED by Serial1 command";
      } else lastEvent="CALIBRATE_FIRST: press button twice first";
    } else if(cmd.equals("STOP")) {
      enabled=false; pendingCommand="STOP"; stop(); lastEvent="STOPPED by Serial1 command";
    }
  }

  int raycastDistance() {
    Cup best=null; float alongBest=101;
    float dx=cos(radians(heading)),dy=sin(radians(heading));
    for(Cup c:cups) if(!c.delivered && !c.mountain) {
      float vx=c.x-x,vy=c.y-y,along=vx*dx+vy*dy,side=abs(vx*dy-vy*dx);
      if(along>0 && along<=100 && side<5.6 && along<alongBest) { best=c; alongBest=along; }
    }
    if(best!=null) { int cupDistance=round(dist(x,y,best.x,best.y)); return best==carriedCup || cupDistance<=2?0:cupDistance; }
    // シミュレーションの超音波反射対象はカップのみ。フィールド境界はエコーとして返さない。
    return -1;
  }

  void tick(float dt) {
    matchTime+=dt;
    distance=raycastDistance();
    floor=surfaceAt(x,y);
    obstacle=distance>0 && distance<OBSTACLE_LIMIT;
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
        if(oldState!=CARRY && state==CARRY && carriedCup==null) {
          carriedCup=nearestCupAhead();
          if(carriedCup==null) {
            float nearest=12;
            for(Cup c:cups) if(!c.delivered && dist(x,y,c.x,c.y)<nearest) { carriedCup=c; nearest=dist(x,y,c.x,c.y); }
          }
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

  void physics(float dt) {
    float vl=left*.075,vr=right*.075,wheelBase=8.5;
    float linear=(vl+vr)*.5,angular=(vr-vl)/wheelBase;
    heading=wrap360(heading+degrees(angular*dt));
    float nx=x+cos(radians(heading))*linear*dt,ny=y+sin(radians(heading))*linear*dt;
    Cup c=nearestCupAhead();
    float cx,cy;
    if(c!=null && linear>0) {
      if(carriedCup==null && (state==APPROACH || state==CARRY)) {
        carriedCup=c;
      } else if(carriedCup!=null) {
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
      cx=x+cos(radians(heading))*7;
      cy=y+sin(radians(heading))*7;
      if(cx< -3) { carriedCup.delivered=true; scoreCup(carriedCup,FLOOR_RED); carriedCup=null; }
      else if(cx>FIELD_W+3) { carriedCup.delivered=true; scoreCup(carriedCup,FLOOR_BLUE); carriedCup=null; }
      else { carriedCup.x=cx; carriedCup.y=cy; }
    }
  }
  Cup nearestCupAhead() {
    Cup best=null; float bd=9;
    float dx=cos(radians(heading)),dy=sin(radians(heading));
    for(Cup c:cups) if(!c.delivered && c!=carriedCup) {
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
  if(!paused) {
    accumulator+=dt*simSpeed;
    while(accumulator>=FIXED_DT) { bot.tick(FIXED_DT); bot.physics(FIXED_DT); accumulator-=FIXED_DT; }
  }
  background(218,228,235);
  drawScene(); drawHud();
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
  drawArena(); drawMountain();
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
  fill(240,70,55); beginShape(TRIANGLES); vertex(7,0,1);vertex(4,-2.5,1);vertex(4,2.5,1);endShape(); popMatrix();
}

void drawHud() {
  float x=18;
  hint(DISABLE_DEPTH_TEST); camera(); ortho(); noLights();
  translate(-width/2,-height/2,0);
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
  text("Control source: Main/*.ino",x+22,302);
  text("Native controller: "+(bot.controllerProcess!=null && bot.controllerProcess.isAlive()?"running":"offline"),x+22,330);
  text("Match time: "+nf(bot.matchTime,1,1)+" s",x+22,358);
  text("Score RED "+scoreRed+"     BLUE "+scoreBlue,x+22,386);
  fill(245,218,120); text("Controls",x+22,427); fill(225);
  text("B / click: button (3 presses starts)",x+22,455);
  text("S: START command     X: STOP command",x+22,481);
  text("Space: pause     R: reset layout",x+22,507);
  text("+ / -: speed x"+nf(simSpeed,1,2)+"     V: sensor ray",x+22,533);
  text("1: black test   2: red test   3: blue test",x+22,559);
  text("Right-drag: move map; view fixed opposite",x+22,585);
  text("Blue Robot 1 start: lower-right",x+22,619);
  text("10 flat cups + 1 hilltop cup (3 pts)",x+22,645);
  text("Hill: 15 cm high, 71 x 70 cm elliptical base",x+22,671);
  text("Main.ino control; approximate 3D physics.",x+22,697);
  if(controllerBuildError.length()>0) { fill(255,150,130); textSize(11); text(controllerBuildError,x+22,715,320,34); }
  hint(ENABLE_DEPTH_TEST);
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
