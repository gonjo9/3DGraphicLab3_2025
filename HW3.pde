import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;

public Vector4 renderer_size;
static public float GH_FOV = 45.0f;
static public float GH_NEAR_MIN = 1e-3f;
static public float GH_NEAR_MAX = 1e-1f;
static public float GH_FAR = 1000.0f;

public boolean debug = true;

public float[] GH_DEPTH;
public PImage renderBuffer;

Engine engine;
Camera main_camera;
Vector3 cam_position;
Vector3 lookat;

float camera_yaw = 0;      // Rotation around Y-axis
float camera_pitch = 0;    // Rotation around X-axis
float camera_distance = 10; // Distance from lookat point
boolean mouseControlActive = false;
int lastMouseX, lastMouseY;

void setup() {
    size(1000, 600);
    renderer_size = new Vector4(20, 50, 520, 550);
    cam_position = new Vector3(0, 0, -10);
    lookat = new Vector3(0, 0, 0);
    setDepthBuffer();
    main_camera = new Camera();
    engine = new Engine();

}

void setDepthBuffer(){
    renderBuffer = new PImage(int(renderer_size.z - renderer_size.x) , int(renderer_size.w - renderer_size.y));
    GH_DEPTH = new float[int(renderer_size.z - renderer_size.x) * int(renderer_size.w - renderer_size.y)];
    for(int i = 0 ; i < GH_DEPTH.length;i++){
        GH_DEPTH[i] = 1.0;
        renderBuffer.pixels[i] = color(1.0*250);
    }
}

void draw() {
    background(255);

    engine.run();
    cameraControl();
}

String selectFile() {
    JFileChooser fileChooser = new JFileChooser();
    fileChooser.setCurrentDirectory(new File("."));
    fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
    FileNameExtensionFilter filter = new FileNameExtensionFilter("Obj Files", "obj");
    fileChooser.setFileFilter(filter);

    int result = fileChooser.showOpenDialog(null);
    if (result == JFileChooser.APPROVE_OPTION) {
        String filePath = fileChooser.getSelectedFile().getAbsolutePath();
        return filePath;
    }
    return "";
}

void cameraControl(){
    // You can write your own camera control function here.
    // Use setPositionOrientation(Vector3 position,Vector3 lookat) to modify the ViewMatrix.
    // Hint : Use keyboard event and mouse click event to change the position of the camera.
    
    // Keyboard controls - move camera and lookat together (orbit mode)
    if (keyPressed) {
        float moveSpeed = 0.1;
        
        // Calculate camera's local axes
        Vector3 forward = lookat.sub(cam_position);
        forward.normalize();
        Vector3 right = Vector3.cross(forward, new Vector3(0, 1, 0));
        right.normalize();
        Vector3 up = Vector3.cross(right, forward);
        up.normalize();
        
        // WASD - move camera and lookat together
        if (key == 'w' || key == 'W') {
            cam_position = cam_position.add(forward.mult(moveSpeed));
            lookat = lookat.add(forward.mult(moveSpeed));
        }
        if (key == 's' || key == 'S') {
            cam_position = cam_position.sub(forward.mult(moveSpeed));
            lookat = lookat.sub(forward.mult(moveSpeed));
        }
        if (key == 'a' || key == 'A') {
            cam_position = cam_position.sub(right.mult(moveSpeed));
            lookat = lookat.sub(right.mult(moveSpeed));
        }
        if (key == 'd' || key == 'D') {
            cam_position = cam_position.add(right.mult(moveSpeed));
            lookat = lookat.add(right.mult(moveSpeed));
        }
        if (key == 'q' || key == 'Q') {
            cam_position = cam_position.add(up.mult(moveSpeed));
            lookat = lookat.add(up.mult(moveSpeed));
        }
        if (key == 'e' || key == 'E') {
            cam_position = cam_position.sub(up.mult(moveSpeed));
            lookat = lookat.sub(up.mult(moveSpeed));
        }
        
        // Update camera distance based on current position
        camera_distance = distance(cam_position, lookat);
    }
    
    main_camera.setPositionOrientation(cam_position, lookat);
}

// Mouse control for camera rotation (orbit around lookat)
void mousePressed() {
    if (mouseButton == LEFT && 
        mouseX >= renderer_size.x && mouseX <= renderer_size.z && 
        mouseY >= renderer_size.y && mouseY <= renderer_size.w) {
        mouseControlActive = true;
        lastMouseX = mouseX;
        lastMouseY = mouseY;
    }
}

void mouseReleased() {
    if (mouseButton == LEFT) {
        mouseControlActive = false;
    }
}

void mouseDragged() {
    if (mouseControlActive) {
        float sensitivity = 0.01;
        camera_yaw += (mouseX - lastMouseX) * sensitivity;
        camera_pitch -= (mouseY - lastMouseY) * sensitivity;
        
        // Clamp pitch to prevent flipping
        camera_pitch = constrain(camera_pitch, -PI/2 + 0.1, PI/2 - 0.1);
        
        // Recalculate camera position based on spherical coordinates
        float camX = lookat.x + camera_distance * sin(camera_yaw) * cos(camera_pitch);
        float camY = lookat.y + camera_distance * sin(camera_pitch);
        float camZ = lookat.z + camera_distance * cos(camera_yaw) * cos(camera_pitch);
        
        cam_position = new Vector3(camX, camY, camZ);
        
        lastMouseX = mouseX;
        lastMouseY = mouseY;
    }
}

void mouseWheel(MouseEvent event) {
    float e = event.getCount();
    camera_distance += e * 0.5;
    camera_distance = constrain(camera_distance, 1.0, 50.0);
    
    // Update camera position based on new distance
    float camX = lookat.x + camera_distance * sin(camera_yaw) * cos(camera_pitch);
    float camY = lookat.y + camera_distance * sin(camera_pitch);
    float camZ = lookat.z + camera_distance * cos(camera_yaw) * cos(camera_pitch);
    
    cam_position = new Vector3(camX, camY, camZ);
}