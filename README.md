# AutoCol - Collision Box Generator

AutoCol is a Blender add-on to quickly generate collision boxes (AABB and OBB) for your models. It works with Blender 5.0.

---

## 🛠️ Installation

1. Download the **autocol** folder.
2. Make sure `autocol_core.dll` is inside that folder.
3. Zip the **autocol** folder.
4. In Blender, go to **Edit > Preferences > Add-ons**.
5. Click **Install...** and select your `.zip` file.
6. Enable **AutoCol - Collision Box Generator**.

---

## 📖 How to Use

1. **Select** your mesh in the 3D Viewport.
2. Open the **Sidebar** (`N` key) and find the **AutoCol** tab.
3. Choose a **Mode**:
   - **Part-Based**: Best for objects with multiple parts (like a chair). 
     - *Check **Oriented Bounding Box** for non-aligned parts.*
   - **K-Means**: Best for complex single meshes.
   - **Single OBB**: One box for the whole object.
4. Click **Generate Collision**.
5. Use **Remove All** to clear generated boxes.

## 🔄 Orientation Settings

You can now choose the **transform orientation** used for collision generation:

- **Local** – Uses each object's local orientation  
- **Global** – Uses world/global axes  
- **Custom (Active)** – Uses the orientation of the active object you have created  

> ⚠️ **Important:**  
> Choosing the correct orientation is essential for generating straight collision boxes.  
> If the object's pivot is not properly aligned (or rotated incorrectly relative to the mesh or group), the generated boxes may appear **skewed or misaligned**.


---

## 🎮 Game Engine Support
- **Prefix**: Change the prefix (e.g., `UCX_`) for Unreal Engine or other...
- **Sollumz Material**: Apply GTA V collision materials directly if you have Sollumz installed.
