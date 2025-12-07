public class Engine {
    Renderer renderer;
    Inspector inspector;
    Hierarchy hierarchy;

    Vector3[] boundary = { new Vector3(-1, -1, 0), new Vector3(-1, 1, 0), new Vector3(1, 1, 0), new Vector3(1, -1, 0) };

    ArrayList<ShapeButton> shapeButton = new ArrayList<ShapeButton>();
    ShapeButton selctFileButton;
    ShapeButton DegubButton;

    public Engine(){
        renderer = new Renderer();
        inspector = new Inspector();
        hierarchy = new Hierarchy(renderer.gameObject);
        main_camera.setSize(int(renderer_size.z - renderer_size.x) , int(renderer_size.w - renderer_size.y) , GH_NEAR_MAX , GH_FAR);
        main_camera.setPositionOrientation(cam_position,lookat);
        initButton();
        
    }

    public void initButton() {
        selctFileButton = new ShapeButton(20, 10, 30, 30) {

        };

        selctFileButton.setBoxAndClickColor(color(250), color(150));
        selctFileButton.setImage(loadImage("cube.png"));
        shapeButton.add(selctFileButton);

        DegubButton = new ShapeButton(60, 10, 30, 30);
        DegubButton.setBoxAndClickColor(color(250), color(150));
        DegubButton.setImage(loadImage("debug.png"));

    }

    void run() {
        setDepthBuffer();
        renderer.run();
        inspector.run();
        hierarchy.run();

        for (ShapeButton sb : shapeButton) {
            sb.run(() -> {
                String path = selectFile();
                // try{
                renderer.addGameObject(new GameObject(path));
                // }catch(Exception ex){
                // println("Occure some error. Please change another files");
                // }
            });
        }

        DegubButton.run(() -> {
            debug = !debug;
        });

    }

}
