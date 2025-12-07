public class Transform {
    Vector3 position;
    Vector3 rotation;
    Vector3 scale;

    public Transform() {
        position = new Vector3();
        rotation = new Vector3();
        scale = new Vector3(1, 1, 1);
    }

    public Transform(Vector3 pos, Vector3 rot, Vector3 scl) {
        position = pos;
        rotation = rot;
        scale = scl;
    }
}
