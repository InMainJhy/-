# 沙发模型升级交付说明

## 交付内容

**网格（OBJ，含 UV）**
- `assets/furniture/sofa_2seat_upholstery_lod0.obj`
- `assets/furniture/sofa_2seat_upholstery_lod1.obj`
- `assets/furniture/sofa_2seat_upholstery_lod2.obj`
- `assets/furniture/sofa_2seat_legs_lod0.obj`
- `assets/furniture/sofa_2seat_legs_lod1.obj`
- `assets/furniture/sofa_2seat_legs_lod2.obj`
- `assets/furniture/armchair_upholstery_lod0.obj`
- `assets/furniture/armchair_upholstery_lod1.obj`
- `assets/furniture/armchair_upholstery_lod2.obj`
- `assets/furniture/armchair_legs_lod0.obj`
- `assets/furniture/armchair_legs_lod1.obj`
- `assets/furniture/armchair_legs_lod2.obj`

**贴图（PBR 纹理集，PNG）**
- `assets/furniture/textures/sofa_fabric_albedo.png`
- `assets/furniture/textures/sofa_fabric_normal.png`
- `assets/furniture/textures/sofa_fabric_roughness.png`
- `assets/furniture/textures/sofa_fabric_ao.png`
- `assets/furniture/textures/armchair_fabric_albedo.png`
- `assets/furniture/textures/armchair_fabric_normal.png`
- `assets/furniture/textures/armchair_fabric_roughness.png`
- `assets/furniture/textures/armchair_fabric_ao.png`

## 在当前仿真模型中的使用方式（MJCF）

已在 [cafe_building.xml](file:///d:/model2.xml/cafe_building.xml) 中完成：
- 新增 mesh/texture/material 资产定义
- 沙发/单人椅可视几何替换为 mesh（软包与木腿分离材质）
- 原有几何保留为碰撞与可压缩仿真，但已设置为透明以避免方块外观

如需切换 LOD，把对应 `geom` 的 `mesh="...lod0"` 改为 `...lod1` 或 `...lod2` 即可。

## OBJ → FBX 导出

仓库提供 Blender 导出脚本：`tools/obj_to_fbx_blender.py`

在安装 Blender 的机器上执行（示例）：
```bash
blender -b -P tools/obj_to_fbx_blender.py -- --input assets/furniture/sofa_2seat_upholstery_lod0.obj --output exports/sofa_2seat_upholstery_lod0.fbx
```

