# Computer Graphics HW3 - 3D Rendering Pipeline

## 1. Rotation Matrix (Y-axis)

**核心算法：** 繞 Y 軸旋轉時，Y 分量保持不變，只旋轉 X 和 Z。

```java
void makeRotY(float a) {
    makeIdentity();
    float c = cos(a);
    float s = sin(a);
    
    m[0]  =  c;  // cos(θ)
    m[2]  =  s;  // sin(θ)
    m[8]  = -s;  // -sin(θ)
    m[10] =  c;  // cos(θ)
}
```

**重點：**
- Y 軸保持不變，旋轉發生在 XZ 平面上
- 使用右手定則：從 Y 軸正方向往下看，逆時針為正旋轉
- 矩陣索引對應到 4x4 陣列的正確位置

---

## 2. Rotation Matrix (X-axis)

**核心算法：** 繞 X 軸旋轉時，X 分量保持不變，旋轉 Y 和 Z。

```java
void makeRotX(float a) {
    makeIdentity();
    float c = cos(a);
    float s = sin(a);
    
    m[5]  = c;   // cos(θ)
    m[6]  = -s;  // -sin(θ)
    m[9]  = s;   // sin(θ)
    m[10] = c;   // cos(θ)
}
```

**重點：**
- X 軸固定，旋轉在 YZ 平面進行
- 三角函數的正負號必須配合右手座標系
- 和 2D 旋轉矩陣的概念相同，只是應用在不同平面

---

## 3. Model Transformation (Model Matrix)

**核心算法：** 將物件從局部空間轉換到世界空間，按照正確順序組合所有變換矩陣。

```java
Matrix4 localToWorld() {
    return Matrix4.Trans(transform.position)
            .mult(Matrix4.RotY(transform.rotation.y))
            .mult(Matrix4.RotX(transform.rotation.x))
            .mult(Matrix4.RotZ(transform.rotation.z))
            .mult(Matrix4.Scale(transform.scale));
}
```

**重點：**
- 變換順序：**T × Ry × Rx × Rz × S** (平移 × 旋轉 × 縮放)
- 矩陣相乘的順序很重要！不能隨便調換
- 錯誤的順序會導致：縮放影響到平移距離、旋轉軸歪掉等問題
- 這個順序保證變換都在物件的局部空間進行，最後才移到世界空間

---

## 4. Camera Transformation (View Matrix)

**核心算法：** 建立 Look-At 相機，把世界空間轉換到相機空間。

```java
void setPositionOrientation(Vector3 pos, Vector3 lookat) {
    Vector3 up = new Vector3(0, 1, 0);
    
    // 計算相機座標系的基底向量
    Vector3 forward = lookat.sub(pos);
    forward.normalize();
    
    Vector3 right = Vector3.cross(forward, up);
    right.normalize();
    
    Vector3 newUp = Vector3.cross(right, forward);
    newUp.normalize();
    
    // 因為 OpenGL 慣例，forward 要取負號
    forward = forward.mult(-1);
    
    // 建構視圖矩陣
    worldView.m[0] = right.x;    worldView.m[1] = right.y;    worldView.m[2] = right.z;
    worldView.m[4] = newUp.x;    worldView.m[5] = newUp.y;    worldView.m[6] = newUp.z;
    worldView.m[8] = forward.x;  worldView.m[9] = forward.y;  worldView.m[10] = forward.z;
    
    // 平移部分
    worldView.m[3] = -Vector3.dot(right, pos);
    worldView.m[7] = -Vector3.dot(newUp, pos);
    worldView.m[11] = -Vector3.dot(forward, pos);
    worldView.m[15] = 1;
}
```

**重點：**
- Forward 向量：從相機指向目標點（之後取負號符合 OpenGL 慣例）
- Right 向量：forward 和世界 up 向量做外積
- Up 向量：right 和 forward 做外積（重新正交化）
- 視圖矩陣其實就是「相機模型矩陣的反矩陣」
- 平移部分要用點積再取負號，把世界原點移到相機位置

---

## 5. Perspective Rendering (Projection Matrix)

**核心算法：** 建立透視投影矩陣，把 3D 視錐體映射到標準化裝置座標 (NDC)。

```java
void setSize(int w, int h, float n, float f) {
    float aspect = (float)w / (float)h;
    float fovRad = radians(GH_FOV);
    float tanHalfFov = tan(fovRad / 2.0);
    
    projection.makeZero();
    projection.m[0] = 1.0 / (aspect * tanHalfFov);
    projection.m[5] = 1.0 / tanHalfFov;
    projection.m[10] = -(far + near) / (far - near);
    projection.m[11] = -(2.0 * far * near) / (far - near);
    projection.m[14] = -1.0;
    projection.m[15] = 0.0;
}
```

**重點：**
- m[0]：X 軸縮放（考慮螢幕長寬比）
- m[5]：Y 軸縮放（由視野角度決定）
- m[10], m[11]：Z 座標的非線性映射
- m[14]：透視除法的關鍵（-1 代表右手座標系）
- 透視投影會讓遠處的東西看起來變小
- 最後會把 3D 空間壓縮到 [-1, 1]³ 的立方體裡

---

## 6. Depth Buffer

**核心算法：** 用重心座標插值法算出三角形內任意點的深度值。

```java
public float getDepth(float x, float y, Vector3[] vertex) {
    Vector3 v0 = vertex[0];
    Vector3 v1 = vertex[1];
    Vector3 v2 = vertex[2];
    
    // 計算重心座標
    float v0v1_x = v1.x - v0.x;
    float v0v1_y = v1.y - v0.y;
    float v0v2_x = v2.x - v0.x;
    float v0v2_y = v2.y - v0.y;
    float v0p_x = x - v0.x;
    float v0p_y = y - v0.y;
    
    float denom = v0v1_x * v0v2_y - v0v2_x * v0v1_y;
    
    if (Math.abs(denom) < 1e-10) {
        return v0.z;  // 退化三角形
    }
    
    float u = (v0p_x * v0v2_y - v0v2_x * v0p_y) / denom;
    float v = (v0v1_x * v0p_y - v0p_x * v0v1_y) / denom;
    float w = 1.0f - u - v;
    
    // 用重心座標插值深度
    float z = w * v0.z + u * v1.z + v * v2.z;
    return z;
}
```

**重點：**
- 計算重心座標 (u, v, w)，代表點到三個頂點的相對距離
- 三個權重加起來必須等於 1：w = 1 - u - v
- 用加權平均插值深度：z = w·z₀ + u·z₁ + v·z₂
- 重心座標讓我們可以在三角形內部平滑插值任何屬性
- 深度緩衝解決了「誰在前面」的問題，只畫最靠近相機的像素

---

## 7. Camera Control

**核心算法：** 實作第一人稱飛行相機，鍵盤控制移動，滑鼠控制視角。

```java
void cameraControl() {
    if (keyPressed) {
        float moveSpeed = 0.1;
        
        // 計算相機的局部座標軸
        Vector3 forward = lookat.sub(cam_position);
        forward.normalize();
        Vector3 right = Vector3.cross(forward, new Vector3(0, 1, 0));
        right.normalize();
        Vector3 up = Vector3.cross(right, forward);
        up.normalize();
        
        // WASD 同時移動相機和目標點
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
        
        camera_distance = distance(cam_position, lookat);
    }
    
    main_camera.setPositionOrientation(cam_position, lookat);
}
```

**滑鼠控制：**

```java
void mouseDragged() {
    if (mouseControlActive) {
        float sensitivity = 0.01;
        camera_yaw += (mouseX - lastMouseX) * sensitivity;
        camera_pitch -= (mouseY - lastMouseY) * sensitivity;
        
        // 限制俯仰角，避免翻轉
        camera_pitch = constrain(camera_pitch, -PI/2 + 0.1, PI/2 - 0.1);
        
        // 用球座標重新計算相機位置
        float camX = lookat.x + camera_distance * sin(camera_yaw) * cos(camera_pitch);
        float camY = lookat.y + camera_distance * sin(camera_pitch);
        float camZ = lookat.z + camera_distance * cos(camera_yaw) * cos(camera_pitch);
        
        cam_position = new Vector3(camX, camY, camZ);
        
        lastMouseX = mouseX;
        lastMouseY = mouseY;
    }
}
```

**重點：**
- W/S：前進/後退
- A/D：左右平移
- Q/E：上升/下降
- 滑鼠拖曳（渲染區域內）：繞著目標點旋轉視角
- 滾輪：拉近/拉遠
- 一開始實作時感覺很怪，因為只移動目標點會讓物體看起來在動
- 解決方法：鍵盤移動時，相機和目標點一起動，保持相對位置

---

## 8. Backface Culling

**核心算法：** 用 2D 叉積判斷三角形朝向，跳過背對相機的面。

```java
void debugDraw() {
    Matrix4 MVP = main_camera.Matrix().mult(localToWorld());
    for (int i = 0; i < mesh.triangles.size(); i++) {
        Triangle triangle = mesh.triangles.get(i);
        Vector3[] img_pos = new Vector3[3];
        for (int j = 0; j < 3; j++) {
            img_pos[j] = MVP.mult(triangle.verts[j].getVector4(1.0)).homogenized();
        }

        // 背面剔除
        Vector3 edge1 = img_pos[1].sub(img_pos[0]);
        Vector3 edge2 = img_pos[2].sub(img_pos[0]);
        
        float crossZ = edge1.x * edge2.y - edge1.y * edge2.x;
        
        if (crossZ <= 0) continue; // 跳過背對的三角形
        
        for (int j = 0; j < img_pos.length; j++) {
            img_pos[j] = new Vector3(map(img_pos[j].x, -1, 1, renderer_size.x, renderer_size.z),
                    map(img_pos[j].y, -1, 1, renderer_size.y, renderer_size.w), img_pos[j].z);
        }

        CGLine(img_pos[0].x, img_pos[0].y, img_pos[1].x, img_pos[1].y);
        CGLine(img_pos[1].x, img_pos[1].y, img_pos[2].x, img_pos[2].y);
        CGLine(img_pos[2].x, img_pos[2].y, img_pos[0].x, img_pos[0].y);
    }
}
```

**重點：**
- 在螢幕空間計算三角形兩條邊
- 算 2D 叉積（只要 Z 分量）：crossZ = edge1.x × edge2.y - edge1.y × edge2.x
- 如果 crossZ ≤ 0，代表三角形是逆時針（背面），跳過不畫
- 只渲染正面（順時針繞序）的三角形
- 背面剔除大約可以減少 50% 的渲染量

---

## 一些截圖

![深度緩衝渲染](screenshots/depth_buffer.png)
*用深度緩衝渲染模型 - 越近的地方越亮*

![背面剔除](screenshots/backface_culling.png)
*只畫正面的三角形，背面的都被剔除了*

![相機控制](screenshots/camera_control.png)
*自由視角相機，可以用 WASD 移動和滑鼠旋轉*

![多個視角](screenshots/viewing_angles.png)
*從不同角度觀察模型*