public class Box {
    public Vector3 pos;
    public Vector3 size;
    protected color box_color = color(0);

    public Box(float x, float y, float w, float h) {
        pos = new Vector3(x, y, 0);
        size = new Vector3(w, h, 0);
    }

    public Box(Vector3 p, Vector3 s) {
        pos = p;
        size = s;
    }

    public Box setBoxColor(color c) {
        box_color = c;
        return this;
    }

    public void show() {
        fill(box_color);
        noStroke();
        rect(pos.x, pos.y, size.x, size.y);
    }

    public boolean checkInSide(Vector3 p) {
        if (p.x >= pos.x && p.x <= pos.x + size.x && p.y >= pos.y && p.y <= pos.y + size.y)
            return true;
        return false;
    }

    public boolean checkInSide() {
        if (mouseX >= pos.x && mouseX <= pos.x + size.x && mouseY >= pos.y && mouseY <= pos.y + size.y)
            return true;
        return false;
    }

}

public class Button extends Box {
    protected color click_color;
    protected boolean press = false;
    protected boolean once = false;
    protected PImage image = null;

    public Button(float x, float y, float w, float h) {
        super(x, y, w, h);
    }

    public Button(Vector3 p, Vector3 s) {
        super(p, s);
    }

    public Button setImage(PImage img) {
        image = img;
        return this;
    }

    public void run(ButtonFunction bf) {
        click(bf);
        show();
    }

    @Override
    public void show() {
        if (!press)
            fill(box_color);
        else
            fill(click_color);
        noStroke();
        rect(pos.x, pos.y, size.x, size.y);
        if (image != null)
            image(image, pos.x, pos.y, size.x, size.y);
    }

    public void setClickColor(color c) {
        click_color = c;
    }

    public Button setBoxAndClickColor(color c1, color c2) {
        setBoxColor(c1);
        setClickColor(c2);
        return this;
    }

    public void click(ButtonFunction bf) {

        if (!checkInSide()) {
            return;
        }
        if (mousePressed) {
            press = true;
            if (!once) {
                bf.function();
                once = true;
            }
        } else {
            press = false;
            once = false;
        }
    }

}

public class HierarchyButton extends Button {
    String name;
    GameObject gameObject;

    public HierarchyButton(float x, float y, float w, float h) {
        super(x, y, w, h);
    }

    public HierarchyButton(Vector3 p, Vector3 s) {
        super(p, s);
    }

    @Override
    public void show() {
        super.show();
        textAlign(CENTER, CENTER);
        textSize(15);
        fill(0);
        text(name, pos.x + 100, pos.y + 15);
    }
}

public class ShapeButton extends Button {
    private boolean selected = false;

    public ShapeButton(float x, float y, float w, float h) {
        super(x, y, w, h);
    }

    public ShapeButton(Vector3 p, Vector3 s) {
        super(p, s);
    }

    @Override
    public void show() {
        super.show();
        if (selected) {
            stroke(255, 0, 0);
            noFill();
            rect(pos.x - 2, pos.y - 2, size.x + 4, size.y + 4);
        }
    }

    public void beSelect() {
        setSelected(true);
    }

    public void setSelected(boolean b) {
        selected = b;
    }

    public GameObject renderShape() {
        return null;
    }

}
