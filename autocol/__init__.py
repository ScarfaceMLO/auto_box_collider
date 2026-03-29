"""
AutoCol - Collision Box Generator
Compatible with Blender 4.x and 5.0
"""
bl_info = {
    "name":        "AutoCol - Collision Box Generator",
    "author":      "ScarfaceMLO",
    "version":     (1, 4, 0),
    "blender":     (4, 0, 0),
    "location":    "View3D > Sidebar > AutoCol",
    "description": "Generate tight-fitting collision boxes with robust active orientation support",
    "category":    "Object",
}

import bpy
import bmesh
import mathutils
import ctypes
import os

# ==============================================================================
# 1. DLL INTERFACE (ctypes)
# ==============================================================================
class OBBResult(ctypes.Structure):
    _fields_ = [
        ("center",   ctypes.c_float * 3),
        ("rotation", ctypes.c_float * 9),
        ("extents",  ctypes.c_float * 3),
    ]

_lib = None
def get_lib():
    global _lib
    if _lib: return _lib
    dll_path = os.path.join(os.path.dirname(__file__), "autocol_core.dll")
    if not os.path.isfile(dll_path): return None
    try:
        _lib = ctypes.CDLL(dll_path)

        _lib.compute_obb.restype  = ctypes.c_int
        _lib.compute_obb.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),   ctypes.c_int,
            ctypes.POINTER(OBBResult),
        ]

        _lib.compute_aabb.restype  = ctypes.c_int
        _lib.compute_aabb.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),   ctypes.c_int,
            ctypes.POINTER(OBBResult),
        ]

        _lib.compute_multi_obb.restype  = ctypes.c_int
        _lib.compute_multi_obb.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),   ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(OBBResult),
            ctypes.POINTER(ctypes.c_int),
        ]
    except Exception:
        _lib = None
        return None
    return _lib


# ==============================================================================
# 2. PROPERTIES
# ==============================================================================
def get_sollumz_material_items(self, context):
    items = [("NONE", "None", "No Sollumz material")]
    try:
        from Sollumz.ybn.collision_materials import collisionmats
        for i, mat in enumerate(collisionmats):
            items.append((str(i), mat.ui_name, f"Apply {mat.ui_name} material"))
    except ImportError:
        pass
    return items

class AutoColProperties(bpy.types.PropertyGroup):
    mode: bpy.props.EnumProperty(
        name="Mode",
        items=[
            ('PART_BASED', "Part-Based",  "Split into loose parts in Blender first (best for chairs/complex mechanical)"),
            ('MULTI_BOX',  "K-Means",     "Use DLL smart clustering (best for single-mesh organics/complex solids)"),
            ('SINGLE_OBB', "Single OBB",  "One oriented bounding box for the whole mesh"),
        ],
        default='PART_BASED',
    )
    orientation_source: bpy.props.EnumProperty(
        name="Orientation",
        items=[
            ('LOCAL',  "Local",  "Use object's local axes"),
            ('GLOBAL', "Global", "Use world axes"),
            ('ACTIVE', "Active", "Use the active transform orientation from viewport header"),
        ],
        default='LOCAL',
        description="Choose the coordinate system for calculating the bounding box"
    )
    max_boxes: bpy.props.IntProperty(
        name="Max Boxes", min=1, max=32, default=5,
        description="For K-Means mode: number of clusters to find"
    )
    naming_prefix: bpy.props.StringProperty(
        name="Prefix", default="UCX",
        description="Prefix for generated collision object names (e.g. UCX for Unreal/FiveM)"
    )
    wire_display: bpy.props.BoolProperty(
        name="Wire Display", default=True,
        description="Display collision boxes as wireframe"
    )
    use_obb: bpy.props.BoolProperty(
        name="Oriented Bounding Box", default=False,
        description="Calculate oriented boxes instead of axis-aligned (best for slanted legs)"
    )
    sollumz_material_type: bpy.props.EnumProperty(
        name="Collision Material",
        items=get_sollumz_material_items,
        description="Sollumz material to apply to generated boxes"
    )


# ==============================================================================
# 3. OPERATORS
# ==============================================================================
class AUTOCOL_OT_generate(bpy.types.Operator):
    bl_idname = "autocol.generate"
    bl_label  = "Generate"
    bl_options = {'UNDO'}

    def get_ref_matrix(self, context, obj, props):
        """Resolves the 4x4 orientation matrix with extreme robustness."""
        mode = props.orientation_source
        
        # 1. Resolve 'ACTIVE' mode using modern transform_orientation_slots
        if mode == 'ACTIVE':
            try:
                slot = context.scene.transform_orientation_slots[0]
                # Check if it's a custom one first
                if slot.custom_orientation:
                    m = slot.custom_orientation.matrix.to_4x4()
                    m.translation = obj.matrix_world.to_translation()
                    return m
                # Otherwise use the type string (GLOBAL, LOCAL, etc.)
                mode = slot.type
            except:
                mode = 'LOCAL'

        # 2. Handle GLOBAL
        if mode == 'GLOBAL':
            m = mathutils.Matrix.Identity(4)
            m.translation = obj.matrix_world.to_translation()
            return m
            
        # 3. Handle LOCAL
        if mode == 'LOCAL':
            return obj.matrix_world

        # 4. Handle Custom by Name (Fallback / Direct choice if orientations exists)
        try:
            # Defensive check for scene.orientations attribute
            if hasattr(context.scene, "orientations"):
                orient = context.scene.orientations.get(mode)
                if orient:
                    m = orient.matrix.to_4x4()
                    m.translation = obj.matrix_world.to_translation()
                    return m
        except:
            pass
            
        # Default fallback to object local axes
        return obj.matrix_world

    def generate_for_obj(self, context, obj, lib, props, idx_offset=0, method='OBB'):
        """Helper to generate a collision box for a specific mesh object."""
        depsgraph = context.evaluated_depsgraph_get()
        obj_eval  = obj.evaluated_get(depsgraph)
        mesh      = obj_eval.to_mesh()

        bm = bmesh.new()
        bm.from_mesh(mesh)
        bmesh.ops.triangulate(bm, faces=bm.faces)
        
        m_ref    = self.get_ref_matrix(context, obj, props)
        inv_ref  = m_ref.inverted()
        world_mat = obj.matrix_world

        verts = []
        for v in bm.verts:
            v_world = world_mat @ v.co
            v_ref   = inv_ref @ v_world
            verts.extend([v_ref.x, v_ref.y, v_ref.z])

        tris = []
        for f in bm.faces:
            for v in f.verts:
                tris.append(v.index)

        bm.free()
        obj_eval.to_mesh_clear()

        if not verts or not tris:
            return 0

        c_verts = (ctypes.c_float * len(verts))(*verts)
        c_tris  = (ctypes.c_int   * len(tris))(*tris)
        nv      = len(verts) // 3
        nt      = len(tris)  // 3

        res = OBBResult()
        func = lib.compute_aabb if method == 'AABB' else lib.compute_obb
        
        if func(c_verts, nv, c_tris, nt, ctypes.byref(res)) == 0:
            self.create_box(context, res, obj, m_ref, props, idx_offset + 1)
            return 1
        return 0

    def execute(self, context):
        self.target_obj = context.active_object
        if not self.target_obj or self.target_obj.type != 'MESH':
            self.report({'ERROR'}, "Select a mesh object first")
            return {'CANCELLED'}

        props = context.scene.autocol_props
        lib   = get_lib()
        if not lib:
            self.report({'ERROR'}, "DLL not found: autocol_core.dll is missing")
            return {'CANCELLED'}

        if props.mode == 'PART_BASED':
            bpy.ops.object.duplicate()
            temp_obj = context.active_object
            temp_obj.name = "AUTOCOL_TEMP"
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.remove_doubles(threshold=0.0001)
            bpy.ops.mesh.tris_convert_to_quads(face_threshold=1.5708, shape_threshold=1.5708, sharp=True)
            bpy.ops.mesh.separate(type='LOOSE')
            bpy.ops.object.mode_set(mode='OBJECT')
            parts = [o for o in context.selected_objects if o != self.target_obj]
            count = 0
            method = 'OBB' if props.use_obb else 'AABB'
            for i, p in enumerate(parts):
                count += self.generate_for_obj(context, p, lib, props, i, method=method)
            bpy.ops.object.select_all(action='DESELECT')
            for p in parts:
                p.select_set(True)
            bpy.ops.object.delete()
            self.target_obj.select_set(True)
            context.view_layer.objects.active = self.target_obj
            self.report({'INFO'}, f"Generated {count} collision boxes from loose parts")

        elif props.mode == 'SINGLE_OBB':
            self.generate_for_obj(context, self.target_obj, lib, props, method='OBB')
        
        else: # MULTI_BOX (K-Means)
            depsgraph = context.evaluated_depsgraph_get()
            obj_eval  = self.target_obj.evaluated_get(depsgraph)
            mesh      = obj_eval.to_mesh()
            bm = bmesh.new()
            bm.from_mesh(mesh)
            bmesh.ops.triangulate(bm, faces=bm.faces)
            m_ref    = self.get_ref_matrix(context, self.target_obj, props)
            inv_ref  = m_ref.inverted()
            world_mat = self.target_obj.matrix_world
            verts = []
            for v in bm.verts:
                v_world = world_mat @ v.co
                v_ref   = inv_ref @ v_world
                verts.extend([v_ref.x, v_ref.y, v_ref.z])
            tris = []
            for f in bm.faces:
                for v in f.verts:
                    tris.append(v.index)
            bm.free()
            obj_eval.to_mesh_clear()
            if not verts or not tris:
                self.report({'WARNING'}, "Object has no geometry")
                return {'CANCELLED'}
            c_verts = (ctypes.c_float * len(verts))(*verts)
            c_tris  = (ctypes.c_int   * len(tris))(*tris)
            nv, nt  = len(verts) // 3, len(tris) // 3
            out   = (OBBResult * props.max_boxes)()
            count = ctypes.c_int(0)
            ret   = lib.compute_multi_obb(c_verts, nv, c_tris, nt,
                                          props.max_boxes, out, ctypes.byref(count))
            if ret == 0 and count.value > 0:
                for i in range(count.value):
                    self.create_box(context, out[i], self.target_obj, m_ref, props, i + 1)
                self.report({'INFO'}, f"Generated {count.value} collision boxes (K-Means)")
            else:
                self.report({'ERROR'}, "K-Means decomposition failed")
                return {'CANCELLED'}

        return {'FINISHED'}

    def get_or_create_collection(self, context, name):
        col = bpy.data.collections.get(name)
        if not col:
            col = bpy.data.collections.new(name)
            context.scene.collection.children.link(col)
        return col

    def create_box(self, context, obb, parent, m_ref, props, idx=1):
        col = self.get_or_create_collection(context, "collider_box_col")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        box      = context.active_object
        target_name = getattr(self, "target_obj", parent).name
        box.name = f"{props.naming_prefix}_{target_name}_{idx:02d}"
        if box.name not in col.objects:
            col.objects.link(box)
        for c in list(box.users_collection):
            if c != col:
                c.objects.unlink(box)
        r = obb.rotation
        rot_mat = mathutils.Matrix((
            (r[0], r[1], r[2]),
            (r[3], r[4], r[5]),
            (r[6], r[7], r[8]),
        ))
        local_mat = mathutils.Matrix.LocRotScale(
            mathutils.Vector((obb.center[0], obb.center[1], obb.center[2])),
            rot_mat,
            mathutils.Vector((obb.extents[0], obb.extents[1], obb.extents[2]))
        )
        box.matrix_world = m_ref @ local_mat
        if props.wire_display:
            box.display_type = 'WIRE'
        if props.sollumz_material_type != "NONE":
            try:
                from Sollumz.ybn.collision_materials import create_collision_material_from_index
                mat = create_collision_material_from_index(int(props.sollumz_material_type))
                if not box.data.materials:
                    box.data.materials.append(mat)
                else:
                    box.data.materials[0] = mat
            except: pass


class AUTOCOL_OT_clear(bpy.types.Operator):
    bl_idname = "autocol.clear"
    bl_label  = "Remove"
    def execute(self, context):
        prefix = context.scene.autocol_props.naming_prefix
        to_remove = [o for o in bpy.data.objects if o.name.startswith(prefix)]
        for o in to_remove:
            bpy.data.objects.remove(o, do_unlink=True)
        return {'FINISHED'}

class AUTOCOL_PT_main(bpy.types.Panel):
    bl_label       = "AutoCol"
    bl_idname      = "AUTOCOL_PT_main"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = "AutoCol"
    def draw(self, context):
        layout = self.layout
        props  = context.scene.autocol_props
        layout.label(text="AutoCol v1.6.3", icon='WORLD')
        if get_lib() is None:
            layout.box().label(text="❌ DLL NOT FOUND!", icon='ERROR')
            return
        box = layout.box()
        box.prop(props, "mode")
        box.prop(props, "orientation_source")
        if props.mode == 'MULTI_BOX':
            box.prop(props, "max_boxes")
        elif props.mode == 'PART_BASED':
            box.prop(props, "use_obb")
        box.prop(props, "naming_prefix")
        box.prop(props, "wire_display")
        box.prop(props, "sollumz_material_type")
        layout.separator()
        row = layout.row(align=True)
        row.scale_y = 2.0
        row.operator("autocol.generate", icon='MOD_PHYSICS', text="Generate Collision")
        row.operator("autocol.clear",    icon='TRASH',       text="Remove All")

classes = [AutoColProperties, AUTOCOL_OT_generate, AUTOCOL_OT_clear, AUTOCOL_PT_main]
def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.autocol_props = bpy.props.PointerProperty(type=AutoColProperties)
def unregister():
    del bpy.types.Scene.autocol_props
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
if __name__ == "__main__": register()
