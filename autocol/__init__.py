"""
AutoCol - Collision Box Generator
Compatible with Blender 4.x and 5.0
"""
bl_info = {
    "name":        "AutoCol - Collision Box Generator",
    "author":      "AutoCol",
    "version":     (1, 6, 7),
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
import importlib
import sys

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
            ctypes.c_int,
            ctypes.POINTER(OBBResult),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]
    except Exception:
        _lib = None
        return None
    return _lib


# ==============================================================================
# 2. SOLLUMZ INTEGRATION HELPERS
# ==============================================================================
def get_sollumz_module_name():
    """Dynamically resolve the Sollumz module name, supporting legacy and extensions."""
    # 1. Try common module names already in sys.modules
    for name in ["Sollumz", "sollumz"]:
        if name in sys.modules:
            return name
            
    # 2. Search sys.modules for Blender extension patterns
    for key in sys.modules.keys():
        if key.lower().endswith(".sollumz") or key.lower().endswith(".sollumz_dev"):
            return key
            
    # 3. Try to import common names directly
    for name in ["Sollumz", "sollumz"]:
        try:
            importlib.import_module(name)
            return name
        except (ImportError, ModuleNotFoundError):
            continue
            
    return None

def get_sollumz_collision_materials():
    """Dynamically import the Sollumz collision_materials module."""
    mod_name = get_sollumz_module_name()
    if not mod_name:
        return None
    try:
        return importlib.import_module(f"{mod_name}.ybn.collision_materials")
    except (ImportError, ModuleNotFoundError):
        return None


# ==============================================================================
# 3. PROPERTIES
# ==============================================================================
def get_sollumz_material_items(self, context):
    items = [("NONE", "None", "No Sollumz material")]
    sollumz_col_mats = get_sollumz_collision_materials()
    if sollumz_col_mats:
        for i, mat in enumerate(sollumz_col_mats.collisionmats):
            items.append((str(i), mat.ui_name, f"Apply {mat.ui_name} material"))
    return items

MATERIAL_KEYWORD_MAP = {
    "concrete": "CONCRETE",
    "pavement": "CONCRETE_PAVEMENT",
    "tarmac": "TARMAC",
    "asphalt": "TARMAC",
    "rock": "ROCK",
    "stone": "STONE",
    "brick": "BRICK",
    "sand": "SAND_LOOSE",
    "grass": "GRASS",
    "wood": "WOOD_SOLID_MEDIUM",
    "plank": "WOOD_SOLID_MEDIUM",
    "timber": "WOOD_SOLID_MEDIUM",
    "metal": "METAL_SOLID_MEDIUM",
    "steel": "METAL_SOLID_MEDIUM",
    "iron": "METAL_SOLID_MEDIUM",
    "glass": "GLASS_SHOOT_THROUGH",
    "plastic": "PLASTIC",
    "dirt": "SOIL",
    "soil": "SOIL",
    "mud": "MUD_HARD",
    "gravel": "GRAVEL_SMALL",
    "water": "WATER",
    "blood": "BLOOD",
    "oil": "OIL",
    "petrol": "PETROL",
    "carpet": "CARPET_SOLID",
    "cloth": "CLOTH",
    "leather": "LEATHER",
    "paper": "PAPER",
    "cardboard": "CARDBOARD_BOX",
    "plaster": "PLASTER_SOLID",
    "ceramic": "CERAMIC",
    "tile": "ROOF_TILE",
    "felt": "ROOF_FELT",
    "rubber": "RUBBER",
    "bark": "TREE_BARK",
    "leave": "LEAVES",
    "bush": "BUSHES",
    "twig": "TWIGS",
    "hay": "HAY",
    "snow": "SNOW_LOOSE",
    "ice": "ICE",
}

def find_collision_material_index_from_parent(obj, mat_indices=None):
    if not obj or obj.type != 'MESH':
        return None
    
    sollumz_col_mats = get_sollumz_collision_materials()
    if not sollumz_col_mats:
        return None
    
    name_to_index = {m.name: i for i, m in enumerate(sollumz_col_mats.collisionmats)}
    
    potential_names = []
    
    # If mat_indices provided, only look at those materials
    if mat_indices is not None:
        for mat_idx in mat_indices:
            if mat_idx < len(obj.material_slots):
                mat = obj.material_slots[mat_idx].material
                if mat:
                    potential_names.append(mat.name.lower())
                    if mat.use_nodes:
                        for node in mat.node_tree.nodes:
                            if node.type == 'TEX_IMAGE' and node.image:
                                potential_names.append(node.image.name.lower())
    else:
        # Fallback to scanning all materials of the object
        for slot in obj.material_slots:
            mat = slot.material
            if mat:
                potential_names.append(mat.name.lower())
                if mat.use_nodes:
                    for node in mat.node_tree.nodes:
                        if node.type == 'TEX_IMAGE' and node.image:
                            potential_names.append(node.image.name.lower())
    
    for name in potential_names:
        for keyword, col_name in MATERIAL_KEYWORD_MAP.items():
            if keyword in name:
                return name_to_index.get(col_name)
    
    return None

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
        name="Max Boxes", min=1, max=100, default=5,
        description="For K-Means mode: total clusters to find"
    )
    precision: bpy.props.IntProperty(
        name="Precision", min=1, max=10, default=1,
        description="Internal precision level (hidden from UI)"
    )
    filter_redundant: bpy.props.BoolProperty(
        name="Avoid Unnecessary Box", default=False,
        description="Automatically remove smaller collision boxes that are entirely contained inside larger ones"
    )
    naming_prefix: bpy.props.StringProperty(
        name="Prefix", default="UCX",
        description="Prefix for generated collision object names (e.g. UCX for Unreal/FiveM)"
    )
    wire_display: bpy.props.BoolProperty(
        name="Wire Display", default=False,
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
    auto_collision_material: bpy.props.BoolProperty(
        name="Auto Collision Material",
        default=False,
        description="Automatically assign collision material based on diffuse texture name"
    )
    auto_sollum_type: bpy.props.BoolProperty(
        name="Set Sollumz Type",
        default=True,
        description="Automatically set generated boxes to Sollumz Bound Poly Box type"
    )
    attach_mode: bpy.props.EnumProperty(
        name="Attach To",
        items=[
            ('NONE',       "None",       "Add to active collection"),
            ('COLLECTION', "Collection", "Add to specific collection"),
            ('OBJECT',     "Object",     "Parent to specific object"),
        ],
        default='COLLECTION',
    )
    attach_collection: bpy.props.PointerProperty(
        name="Collection",
        type=bpy.types.Collection,
    )
    attach_object: bpy.props.PointerProperty(
        name="Object",
        type=bpy.types.Object,
    )
    smart_cylinder: bpy.props.BoolProperty(
        name="Smart Cylinder",
        default=False,
        description="Automatically detect if a part should be a cylinder (poles, etc)"
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
            return []

        c_verts = (ctypes.c_float * len(verts))(*verts)
        c_tris  = (ctypes.c_int   * len(tris))(*tris)
        nv      = len(verts) // 3
        nt      = len(tris)  // 3

        res = OBBResult()
        func = lib.compute_aabb if method == 'AABB' else lib.compute_obb
        
        if func(c_verts, nv, c_tris, nt, ctypes.byref(res)) == 0:
            box = self.create_box(context, res, obj, m_ref, props, idx_offset + 1)
            return [box] if box else []
        return []

    def generate_multi_for_obj(self, context, obj, lib, props, idx_offset=0):
        """Helper to generate multiple collision boxes for a mesh using K-Means."""
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
        
        tri_material_indices = [f.material_index for f in bm.faces]
        bm.free()
        obj_eval.to_mesh_clear()
        
        if not verts or not tris:
            return []
            
        c_verts = (ctypes.c_float * len(verts))(*verts)
        c_tris  = (ctypes.c_int   * len(tris))(*tris)
        nv, nt  = len(verts) // 3, len(tris) // 3
        
        # Decide how many boxes to generate
        k_val = props.precision if props.mode == 'PART_BASED' else props.max_boxes
        use_obb = 1 if props.use_obb else 0
        
        out   = (OBBResult * k_val)()
        count = ctypes.c_int(0)
        c_face_labels = (ctypes.c_int * nt)()
        
        ret = lib.compute_multi_obb(c_verts, nv, c_tris, nt,
                                    k_val, use_obb, out, ctypes.byref(count), c_face_labels)
        
        if ret != 0:
            self.report({'ERROR'}, f"K-Means DLL failed (code {ret})")
            return []
            
        boxes = []
        if count.value > 0:
            # Group material indices by cluster
            clusters_mats = [set() for _ in range(count.value)]
            for t in range(nt):
                cid = c_face_labels[t]
                if 0 <= cid < count.value:
                    clusters_mats[cid].add(tri_material_indices[t])

            for i in range(count.value):
                mat_idx = None
                if props.auto_collision_material:
                    # Pass the original object (with its material slots) for material detection
                    mat_idx = find_collision_material_index_from_parent(obj, clusters_mats[i])
                box = self.create_box(context, out[i], obj, m_ref, props, idx_offset + i + 1, mat_idx)
                if box:
                    boxes.append(box)
            return boxes
        return []

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

        all_boxes = []
        if props.mode == 'PART_BASED':
            # Ensure only the target object is selected before duplicating
            bpy.ops.object.select_all(action='DESELECT')
            self.target_obj.select_set(True)
            context.view_layer.objects.active = self.target_obj

            bpy.ops.object.duplicate()
            temp_obj = context.active_object
            temp_obj.name = "AUTOCOL_TEMP"
            
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.remove_doubles(threshold=0.0001)
            bpy.ops.mesh.tris_convert_to_quads(face_threshold=1.5708, shape_threshold=1.5708, sharp=True)
            bpy.ops.mesh.separate(type='LOOSE')
            bpy.ops.object.mode_set(mode='OBJECT')
            
            # The separated parts plus the original 'temp_obj' (which is now one of the parts)
            parts = [o for o in context.selected_objects if o != self.target_obj]
            
            count = 0
            method = 'OBB' if props.use_obb else 'AABB'
            for i, p in enumerate(parts):
                if p.type != 'MESH':
                    continue
                    
                # Ensure the part is active so operators work on it
                context.view_layer.objects.active = p
                
                # Safety check: material_slot_remove_unused polls False if no material slots exist
                if p.material_slots:
                    try:
                        with context.temp_override(active_object=p, object=p, selected_objects=[p]):
                            if bpy.ops.object.material_slot_remove_unused.poll():
                                bpy.ops.object.material_slot_remove_unused()
                    except Exception: pass
                
                if props.precision > 1:
                    new_boxes = self.generate_multi_for_obj(context, p, lib, props, idx_offset=len(all_boxes))
                else:
                    new_boxes = self.generate_for_obj(context, p, lib, props, idx_offset=len(all_boxes), method=method)
                all_boxes.extend(new_boxes)
            
            if props.filter_redundant:
                removed = self.remove_redundant_boxes(all_boxes)
                if removed > 0:
                    self.report({'INFO'}, f"Cleaned up {removed} redundant boxes")
            
            bpy.ops.object.select_all(action='DESELECT')
            for p in parts: p.select_set(True)
            bpy.ops.object.delete()
            
            self.target_obj.select_set(True)
            context.view_layer.objects.active = self.target_obj
            self.report({'INFO'}, f"Generated {len(all_boxes)} collision boxes from loose parts")

        elif props.mode == 'SINGLE_OBB':
            all_boxes = self.generate_for_obj(context, self.target_obj, lib, props, method='OBB')
        
        else: # MULTI_BOX (K-Means)
            # Temporary cleanup for better K-Means
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.remove_doubles(threshold=0.0001)
            bpy.ops.object.mode_set(mode='OBJECT')
            
            all_boxes = self.generate_multi_for_obj(context, self.target_obj, lib, props)
            
            if all_boxes:
                self.report({'INFO'}, f"Generated {len(all_boxes)} collision boxes (K-Means)")
            else:
                self.report({'ERROR'}, "K-Means decomposition failed")
                return {'CANCELLED'}

        return {'FINISHED'}


    def remove_redundant_boxes(self, boxes):
        """Removes boxes that are entirely contained within other larger boxes."""
        if len(boxes) < 2: return 0
        
        to_delete = set()
        # Potential corner multipliers
        mults = [(mx, my, mz) for mx in (-1, 1) for my in (-1, 1) for mz in (-1, 1)]
        
        # Sort by volume (largest first)
        get_vol = lambda b: b.dimensions.x * b.dimensions.y * b.dimensions.z
        boxes_sorted = sorted(boxes, key=get_vol, reverse=True)
        
        for i, b_outer in enumerate(boxes_sorted):
            if b_outer in to_delete: continue
            
            inv_outer = b_outer.matrix_world.inverted()
            half_outer = b_outer.dimensions / 2.0
            eps = 0.07 # 6cm epsilon - fine-tuning
            
            for j, b_inner in enumerate(boxes_sorted):
                if i == j or b_inner in to_delete: continue
                
                half_inner = b_inner.dimensions / 2.0
                
                # Transform all 8 corners of b_inner to b_outer's local space
                pts_local = []
                for mx, my, mz in mults:
                    v_local = mathutils.Vector((mx * half_inner.x, my * half_inner.y, mz * half_inner.z))
                    v_world = b_inner.matrix_world @ v_local
                    v_outer = inv_outer @ v_world
                    pts_local.append(v_outer)
                
                # Check if the entire local bounding box of b_inner is within b_outer's bounds
                all_inside = True
                for p in pts_local:
                    if (abs(p.x) > half_outer.x + eps or 
                        abs(p.y) > half_outer.y + eps or 
                        abs(p.z) > half_outer.z + eps):
                        all_inside = False
                        break
                
                if all_inside:
                    to_delete.add(b_inner)
        
        count = len(to_delete)
        for b in to_delete:
            if b in boxes: boxes.remove(b)
            bpy.data.objects.remove(b, do_unlink=True)
        return count

    def is_geometry_cylindrical(self, obj, obb):
        """Heuristic to detect if a mesh is more cylindrical than boxy."""
        if not obj or obj.type != 'MESH':
            return False
        
        ext = obb.extents
        # 1. Footprint check: detect the 'round' axis (the one with the largest or smallest extent)
        # Typically the 'height' is the largest, footprint is the other two.
        sorted_indices = sorted(range(3), key=lambda i: ext[i], reverse=True)
        h_idx = sorted_indices[0] # Assume largest is height
        f1, f2 = sorted_indices[1], sorted_indices[2]
        
        # Footprint must be somewhat square
        footprint_ratio = ext[f1] / max(ext[f2], 0.001)
        if footprint_ratio > 1.3:
            return False
            
        # 2. Vertex distribution check (sampling)
        # We check how many vertices are near the corners of the OBB vs the edges of a circle.
        # A cylinder has ~78% the volume of a box.
        # Cheap vertex test: distance to center axis.
        # Radius of cylinder would be avg of footprint extents.
        r_target = (ext[f1] + ext[f2]) / 2.0
        
        # We'll use local vertices and project them to OBB space coordinate system
        # (This is complex to do right without full matrix math, so let's use a simpler heuristic:
        # If it's a pole-like thing (large aspect ratio) and the footprint is square, 
        # it's usually intended to be a cylinder in GTA/Sollumz maps).
        
        aspect_ratio = ext[h_idx] / max(ext[f1], 0.001)
        if aspect_ratio > 2.0: # Tall or wide (pole/disc)
             return True
             
        return False

    def get_or_create_collection(self, context, name):
        col = bpy.data.collections.get(name)
        if not col:
            col = bpy.data.collections.new(name)
            context.scene.collection.children.link(col)
        return col

    def create_box(self, context, obb, parent, m_ref, props, idx=1, override_mat_idx=None):
        # 1. Determine target collection
        target_col = None
        if props.attach_mode == 'COLLECTION':
            target_col = props.attach_collection
            if not target_col:
                target_col = self.get_or_create_collection(context, "collider_box_col")
        elif props.attach_mode == 'OBJECT':
            if props.attach_object:
                if props.attach_object.users_collection:
                    target_col = props.attach_object.users_collection[0]
            if not target_col:
                target_col = context.collection
        else: # NONE
            target_col = context.collection

        # 2. Add the collision object
        is_cyl = False
        if props.mode == 'PART_BASED' and props.smart_cylinder:
            is_cyl = self.is_geometry_cylindrical(parent, obb)

        if is_cyl:
            bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=2.0)
        else:
            bpy.ops.mesh.primitive_cube_add(size=2.0)
            
        box = context.active_object
        
        # 3. Parenting
        if props.attach_mode == 'OBJECT' and props.attach_object:
            box.parent = props.attach_object
            # Note: matrix_world is set below override-style, so parenting is safe here

        # 4. Set Sollumz type (Box or Cylinder)
        if props.auto_sollum_type and hasattr(box, "sollum_type"):
            if is_cyl:
                box.sollum_type = 'sollumz_bound_poly_cylinder'
            else:
                box.sollum_type = 'sollumz_bound_poly_box'

        target_name = getattr(self, "target_obj", parent).name
        box.name = f"{props.naming_prefix}_{target_name}_{idx:02d}"
        
        # 5. Link to target collection and unlink from others
        if box.name not in target_col.objects:
            target_col.objects.link(box)
        for c in list(box.users_collection):
            if c != target_col:
                c.objects.unlink(box)
        r = obb.rotation
        rot_mat = mathutils.Matrix((
            (r[0], r[1], r[2]),
            (r[3], r[4], r[5]),
            (r[6], r[7], r[8]),
        ))
        
        final_rot = rot_mat.copy()
        target_ext = mathutils.Vector((obb.extents[0], obb.extents[1], obb.extents[2]))
        
        if is_cyl:
            # Find the longest axis of the OBB to align the cylinder height (Z) with it
            h_idx = 0
            if obb.extents[1] > obb.extents[h_idx]: h_idx = 1
            if obb.extents[2] > obb.extents[h_idx]: h_idx = 2
            
            if h_idx == 0:
                # X is longest. Rotate local Z to X.
                final_rot = rot_mat @ mathutils.Matrix.Rotation(1.570796, 3, 'Y')
                target_ext = mathutils.Vector((obb.extents[2], obb.extents[1], obb.extents[0]))
            elif h_idx == 1:
                # Y is longest. Rotate local Z to Y.
                final_rot = rot_mat @ mathutils.Matrix.Rotation(1.570796, 3, 'X')
                target_ext = mathutils.Vector((obb.extents[0], obb.extents[2], obb.extents[1]))
                
        local_mat = mathutils.Matrix.LocRotScale(
            mathutils.Vector((obb.center[0], obb.center[1], obb.center[2])),
            final_rot,
            target_ext
        )
        box.matrix_world = m_ref @ local_mat
        
        # 6. Apply scale so it's 1,1,1
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

        if props.wire_display:
            box.display_type = 'WIRE'
        if props.auto_collision_material:
            auto_idx = override_mat_idx if override_mat_idx is not None else find_collision_material_index_from_parent(parent)
            if auto_idx is not None:
                sollumz_col_mats = get_sollumz_collision_materials()
                if sollumz_col_mats:
                    try:
                        mat = sollumz_col_mats.create_collision_material_from_index(auto_idx)
                        if not box.data.materials:
                            box.data.materials.append(mat)
                        else:
                            box.data.materials[0] = mat
                    except: pass
                return box

        if props.sollumz_material_type != "NONE":
            sollumz_col_mats = get_sollumz_collision_materials()
            if sollumz_col_mats:
                try:
                    mat = sollumz_col_mats.create_collision_material_from_index(int(props.sollumz_material_type))
                    if not box.data.materials:
                        box.data.materials.append(mat)
                    else:
                        box.data.materials[0] = mat
                except: pass
        
        return box


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

        # 1. Header & Version
        row = layout.row(align=True)
        row.label(text=f"AutoCol v{bl_info['version'][0]}.{bl_info['version'][1]}.{bl_info['version'][2]}", icon='WORLD_DATA')
        layout.separator()

        # 2. Organization (Attach To) - Top Priority
        box_attach = layout.box()
        box_attach.label(text="Attach To", icon='LINKED')
        box_attach.prop(props, "attach_mode", expand=True)
        if props.attach_mode == 'COLLECTION':
            box_attach.prop(props, "attach_collection", text="", icon='COLLECTION_NEW')
        elif props.attach_mode == 'OBJECT':
            box_attach.prop(props, "attach_object", text="", icon='OBJECT_DATA')
        
        layout.separator()

        # 3. Generation Settings
        box_gen = layout.box()
        box_gen.label(text="Generation Settings", icon='PROPERTIES')
        box_gen.prop(props, "mode")
        
        if props.mode == 'MULTI_BOX':
            box_gen.prop(props, "max_boxes", text="Total Boxes")
            
        box_gen.prop(props, "orientation_source")
        box_gen.prop(props, "naming_prefix")
        if props.mode == 'PART_BASED':
            box_gen.prop(props, "smart_cylinder")
            box_gen.prop(props, "filter_redundant")
            
        box_gen.prop(props, "use_obb")
        box_gen.prop(props, "wire_display")
        
        layout.separator()

        # 4. Sollumz Integration (Optional/Advanced)
        box_sollum = layout.box()
        box_sollum.label(text="Sollumz Options", icon='MATERIAL')
        box_sollum.prop(props, "sollumz_material_type")
        
        row_auto = box_sollum.row()
        row_auto.prop(props, "auto_collision_material")
        row_auto.prop(props, "auto_sollum_type")
        
        layout.separator()

        # 5. Actions
        row_actions = layout.row(align=True)
        row_actions.scale_y = 2.0
        row_actions.operator("autocol.generate", icon='GHOST_ENABLED')
        row_actions.operator("autocol.clear",    icon='TRASH', text="Remove All")

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
