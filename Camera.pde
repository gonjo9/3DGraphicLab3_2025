public class Camera {
    Matrix4 projection = new Matrix4();
    Matrix4 worldView = new Matrix4();
    int wid;
    int hei;
    float near;
    float far;
    Transform transform;

    Camera() {
        wid = 256;
        hei = 256;
        worldView.makeIdentity();
        projection.makeIdentity();
        transform = new Transform();
    }

    Matrix4 inverseProjection() {
        Matrix4 invProjection = Matrix4.Zero();
        float a = projection.m[0];
        float b = projection.m[5];
        float c = projection.m[10];
        float d = projection.m[11];
        float e = projection.m[14];
        invProjection.m[0] = 1.0f / a;
        invProjection.m[5] = 1.0f / b;
        invProjection.m[11] = 1.0f / e;
        invProjection.m[14] = 1.0f / d;
        invProjection.m[15] = -c / (d * e);
        return invProjection;
    }

    Matrix4 Matrix() {
        return projection.mult(worldView);
    }

    void setSize(int w, int h, float n, float f) {
        wid = w;
        hei = h;
        near = n;
        far = f;
        
        // TODO HW3
        // This function takes four parameters, which are 
        // the width of the screen, the height of the screen
        // the near plane and the far plane of the camera.
        // Where GH_FOV has been declared as a global variable.
        // Finally, pass the result into projection matrix.
        float aspect = (float)w / (float)h;
        float fovRad = radians(GH_FOV);
        float tanHalfFov = tan(fovRad / 2.0);
        
        // Perspective projection matrix
        projection.makeZero();
        projection.m[0] = 1.0 / (aspect * tanHalfFov);
        projection.m[5] = 1.0 / tanHalfFov;
        projection.m[10] = -(far + near) / (far - near);
        projection.m[11] = -(2.0 * far * near) / (far - near);
        projection.m[14] = -1.0;
        projection.m[15] = 0.0;




    }

    

    void setPositionOrientation(Vector3 pos, float rotX, float rotY) {

    }

    void setPositionOrientation(Vector3 pos, Vector3 lookat) {
        // TODO HW3
        // This function takes two parameters, which are the position of the camera and
        // the point the camera is looking at.
        // We uses topVector = (0,1,0) to calculate the eye matrix.
        // Finally, pass the result into worldView matrix.

        Vector3 up = new Vector3(0, 1, 0);
        
        // Calculate camera coordinate system
        // Forward: direction from camera to lookat (then negate for right-hand system)
        Vector3 forward = lookat.sub(pos);
        forward.normalize();
        
        // Right: cross product of forward and up
        Vector3 right = Vector3.cross(forward, up);
        right.normalize();
        
        // Recalculate up: cross product of right and forward
        Vector3 newUp = Vector3.cross(right, forward);
        newUp.normalize();
        
        // Negate forward for right-hand coordinate system (camera looks down -Z)
        forward = forward.mult(-1);
        
        // Build view matrix: rotation part
        worldView.m[0] = right.x;    worldView.m[1] = right.y;    worldView.m[2] = right.z;
        worldView.m[4] = newUp.x;    worldView.m[5] = newUp.y;    worldView.m[6] = newUp.z;
        worldView.m[8] = forward.x;  worldView.m[9] = forward.y;  worldView.m[10] = forward.z;
        
        // Translation part: -dot product with position
        worldView.m[3] = -Vector3.dot(right, pos);
        worldView.m[7] = -Vector3.dot(newUp, pos);
        worldView.m[11] = -Vector3.dot(forward, pos);
        worldView.m[12] = 0;         worldView.m[13] = 0;         worldView.m[14] = 0;         worldView.m[15] = 1;
    }
}
